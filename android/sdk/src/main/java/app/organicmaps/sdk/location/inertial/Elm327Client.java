package app.organicmaps.sdk.location.inertial;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.sdk.util.log.Logger;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.UUID;

/**
 * Reads the car speed from an ELM327 Bluetooth (SPP) OBD-II adapter.
 * Works on its own thread, reconnects on errors, reports to the listener on the main thread.
 */
public class Elm327Client
{
  private static final String TAG = Elm327Client.class.getSimpleName();
  private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
  private static final long RECONNECT_DELAY_MS = 3000;
  private static final long RESPONSE_TIMEOUT_MS = 3000;
  // Includes the protocol search on the first request.
  private static final long FIRST_RESPONSE_TIMEOUT_MS = 15000;
  // The adapter replies in ~50-200 ms, don't flood the car bus.
  private static final long REQUEST_INTERVAL_MS = 100;
  // Many adapters don't find the car again after its ignition was off: the adapter is reset then.
  private static final int MAX_NO_DATA_REQUESTS = 3;

  public enum State
  {
    DISCONNECTED,
    CONNECTING,
    // Connected to the adapter, but the car doesn't answer (ignition off, protocol search).
    NO_CAR_DATA,
    CONNECTED,
  }

  public interface Listener
  {
    void onSpeed(int speedKmh, long elapsedRealtimeMs);

    void onStateChanged(@NonNull State state);
  }

  @NonNull
  private final String mAddress;
  @NonNull
  private final Listener mListener;
  private final Handler mMainHandler = new Handler(Looper.getMainLooper());
  @Nullable
  private volatile Thread mThread;
  @Nullable
  private volatile BluetoothSocket mSocket;
  private volatile boolean mRunning;
  @Nullable
  private State mLastState;

  public Elm327Client(@NonNull String address, @NonNull Listener listener)
  {
    mAddress = address;
    mListener = listener;
  }

  public void start()
  {
    if (mRunning)
      return;
    mRunning = true;
    final Thread thread = new Thread(this::run, "Elm327");
    mThread = thread;
    thread.start();
  }

  public void stop()
  {
    mRunning = false;
    closeSocket();
    final Thread thread = mThread;
    if (thread != null)
      thread.interrupt();
    mThread = null;
  }

  private void run()
  {
    while (mRunning)
    {
      try
      {
        setState(State.CONNECTING);
        connect();
        initAdapter();
        pollSpeed();
      }
      catch (IOException | SecurityException e)
      {
        Logger.w(TAG, "ELM327 error: " + e.getMessage());
      }
      catch (InterruptedException e)
      {
        break;
      }
      finally
      {
        closeSocket();
      }

      if (!mRunning)
        break;
      setState(State.DISCONNECTED);
      try
      {
        Thread.sleep(RECONNECT_DELAY_MS);
      }
      catch (InterruptedException e)
      {
        break;
      }
    }
    setState(State.DISCONNECTED);
  }

  @SuppressLint("MissingPermission") // BLUETOOTH_CONNECT is checked by the caller.
  private void connect() throws IOException
  {
    final BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
    if (adapter == null || !adapter.isEnabled())
      throw new IOException("Bluetooth is off");
    final BluetoothDevice device = adapter.getRemoteDevice(mAddress);
    BluetoothSocket socket;
    try
    {
      socket = device.createRfcommSocketToServiceRecord(SPP_UUID);
      mSocket = socket;
      socket.connect();
    }
    catch (IOException e)
    {
      // Many cheap clones work only with an insecure connection.
      closeSocket();
      socket = device.createInsecureRfcommSocketToServiceRecord(SPP_UUID);
      mSocket = socket;
      socket.connect();
    }
    Logger.i(TAG, "Connected to " + mAddress);
  }

  private void initAdapter() throws IOException, InterruptedException
  {
    command("ATZ", RESPONSE_TIMEOUT_MS); // Reset.
    command("ATE0", RESPONSE_TIMEOUT_MS); // Echo off.
    command("ATL0", RESPONSE_TIMEOUT_MS); // Linefeeds off.
    command("ATS0", RESPONSE_TIMEOUT_MS); // Spaces off.
    command("ATH0", RESPONSE_TIMEOUT_MS); // Headers off.
    command("ATSP0", RESPONSE_TIMEOUT_MS); // Automatic protocol.
    setState(State.NO_CAR_DATA);
  }

  private void pollSpeed() throws IOException, InterruptedException
  {
    long timeout = FIRST_RESPONSE_TIMEOUT_MS;
    int noDataRequests = 0;
    while (mRunning)
    {
      final long requestTime = SystemClock.elapsedRealtime();
      final String response = command("010D", timeout);
      final long time = (requestTime + SystemClock.elapsedRealtime()) / 2;
      final int speed = Elm327Parser.parseSpeed(response);
      if (speed == Elm327Parser.NO_SPEED)
      {
        setState(State.NO_CAR_DATA);
        if (++noDataRequests >= MAX_NO_DATA_REQUESTS)
          throw new IOException("No speed from the car: " + response.trim());
        timeout = FIRST_RESPONSE_TIMEOUT_MS;
        Thread.sleep(RECONNECT_DELAY_MS);
        continue;
      }
      noDataRequests = 0;
      timeout = RESPONSE_TIMEOUT_MS;
      setState(State.CONNECTED);
      mMainHandler.post(() -> mListener.onSpeed(speed, time));
      Thread.sleep(REQUEST_INTERVAL_MS);
    }
  }

  /**
   * Sends a command and reads the response up to the '>' prompt.
   */
  @NonNull
  private String command(@NonNull String cmd, long timeoutMs) throws IOException, InterruptedException
  {
    final BluetoothSocket socket = mSocket;
    if (socket == null)
      throw new IOException("Not connected");
    final OutputStream out = socket.getOutputStream();
    final InputStream in = socket.getInputStream();
    out.write((cmd + "\r").getBytes(StandardCharsets.US_ASCII));
    out.flush();

    final StringBuilder response = new StringBuilder();
    final long deadline = SystemClock.elapsedRealtime() + timeoutMs;
    while (SystemClock.elapsedRealtime() < deadline)
    {
      if (in.available() == 0)
      {
        Thread.sleep(10);
        continue;
      }
      final int b = in.read();
      if (b < 0)
        throw new IOException("Connection closed");
      if (b == '>')
        return response.toString();
      response.append((char) b);
    }
    throw new IOException("Timeout on " + cmd);
  }

  private void closeSocket()
  {
    final BluetoothSocket socket = mSocket;
    mSocket = null;
    if (socket == null)
      return;
    try
    {
      socket.close();
    }
    catch (IOException e)
    {
      Logger.w(TAG, "Failed to close the socket: " + e.getMessage());
    }
  }

  private void setState(@NonNull State state)
  {
    if (state == mLastState)
      return;
    mLastState = state;
    Logger.i(TAG, "state = " + state);
    mMainHandler.post(() -> mListener.onStateChanged(state));
  }
}
