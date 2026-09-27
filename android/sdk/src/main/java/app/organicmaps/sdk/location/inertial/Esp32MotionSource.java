package app.organicmaps.sdk.location.inertial;

import android.content.Context;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.sdk.util.log.Logger;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * The NoGPS ESP32 sensor box: it is fixed in the car and sends the car rotation and speed over its own Wi-Fi
 * network, the phone can be held in hands. The box zeroes its gyroscope by itself on every stop.
 */
public class Esp32MotionSource implements MotionSource
{
  private static final String TAG = Esp32MotionSource.class.getSimpleName();

  private static final int BOX_PORT = 4210;
  // The box sends data while it hears the phone.
  private static final long HELLO_INTERVAL_MS = 1000;
  // The box sends 50 lines a second, a longer silence means it is lost.
  private static final long DATA_TIMEOUT_MS = 1000;
  private static final int RECEIVE_TIMEOUT_MS = 200;
  // A calibration takes ~3 s on the box.
  private static final long CALIBRATION_TIMEOUT_MS = 10_000;

  @NonNull
  private final String mAddress;
  @NonNull
  private final Listener mListener;
  @Nullable
  private final ConnectivityManager mConnectivityManager;
  private final Handler mMainHandler = new Handler(Looper.getMainLooper());
  private final Queue<String> mOutgoing = new ConcurrentLinkedQueue<>();
  private volatile boolean mRunning;
  // The thread talking to the box, a stopped one ends by itself.
  @Nullable
  private volatile Thread mThread;
  // The Wi-Fi network of the box. It has no internet, so Android sends everything else over the mobile data,
  // and the sockets to the box must be bound to it.
  @Nullable
  private volatile Network mWifi;
  @Nullable
  private ConnectivityManager.NetworkCallback mNetworkCallback;

  // The state below is used on the main thread only.
  private long mLastDataMs;
  private int mFlags;
  private boolean mHasLast;
  private long mLastTimeMs;
  private long mLastYawMdeg;
  @Nullable
  private String mImuName;
  private int mNextCommandId = 1;
  // The calibration command waiting for its reply, 0 if none.
  private int mCalibrationId;
  private long mCalibrationStartMs;
  private int mCalibrationProgress;
  private boolean mCalibrationFailed;

  /**
   * @param address the address of the box, 192.168.4.1 for its own access point.
   */
  public Esp32MotionSource(@NonNull Context context, @NonNull String address, @NonNull Listener listener)
  {
    mAddress = address;
    mListener = listener;
    mConnectivityManager = (ConnectivityManager) context.getSystemService(Context.CONNECTIVITY_SERVICE);
  }

  @Override
  public void start()
  {
    if (mRunning)
      return;
    Logger.i(TAG, "address = " + mAddress);
    mRunning = true;
    watchWifi();
    final Thread thread = new Thread(this::run, "Esp32");
    mThread = thread;
    thread.start();
  }

  @Override
  public void stop()
  {
    if (!mRunning)
      return;
    Logger.i(TAG);
    mRunning = false;
    final Thread thread = mThread;
    mThread = null;
    if (thread != null)
      thread.interrupt();
    if (mConnectivityManager != null && mNetworkCallback != null)
      mConnectivityManager.unregisterNetworkCallback(mNetworkCallback);
    mNetworkCallback = null;
    mHasLast = false;
    mLastDataMs = 0;
  }

  private void watchWifi()
  {
    if (mConnectivityManager == null)
      return;
    final NetworkRequest request = new NetworkRequest.Builder()
                                       .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                                       .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
                                       .build();
    mNetworkCallback = new ConnectivityManager.NetworkCallback() {
      @Override
      public void onAvailable(@NonNull Network network)
      {
        Logger.i(TAG, "Wi-Fi = " + network);
        mWifi = network;
      }

      @Override
      public void onLost(@NonNull Network network)
      {
        Logger.i(TAG, "Wi-Fi lost = " + network);
        if (network.equals(mWifi))
          mWifi = null;
      }
    };
    mConnectivityManager.registerNetworkCallback(request, mNetworkCallback);
  }

  private boolean isCurrentThread()
  {
    return mRunning && Thread.currentThread() == mThread;
  }

  private void run()
  {
    while (isCurrentThread())
    {
      final Network network = mWifi;
      try (DatagramSocket socket = new DatagramSocket())
      {
        if (network != null && Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP_MR1)
          network.bindSocket(socket);
        socket.setSoTimeout(RECEIVE_TIMEOUT_MS);
        exchange(socket, network);
      }
      catch (IOException e)
      {
        Logger.w(TAG, "Socket error", e);
        sleep(HELLO_INTERVAL_MS);
      }
    }
  }

  /**
   * Talks to the box until the Wi-Fi network changes or the source is stopped.
   */
  private void exchange(@NonNull DatagramSocket socket, @Nullable Network network) throws IOException
  {
    final InetAddress box = InetAddress.getByName(mAddress);
    final byte[] buffer = new byte[512];
    long lastHelloMs = 0;
    int helloId = 0;
    while (isCurrentThread() && network == mWifi)
    {
      final long now = SystemClock.elapsedRealtime();
      if (now - lastHelloMs >= HELLO_INTERVAL_MS)
      {
        lastHelloMs = now;
        send(socket, box, Esp32Protocol.command(++helloId, "HELLO"));
      }
      for (String line = mOutgoing.poll(); line != null; line = mOutgoing.poll())
        send(socket, box, line);

      final DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
      try
      {
        socket.receive(packet);
      }
      catch (SocketTimeoutException e)
      {
        continue;
      }
      final String text = new String(packet.getData(), 0, packet.getLength(), StandardCharsets.US_ASCII);
      final String[] fields = Esp32Protocol.parse(text);
      if (fields != null)
        mMainHandler.post(() -> onLine(fields));
    }
  }

  private static void send(@NonNull DatagramSocket socket, @NonNull InetAddress box, @NonNull String line)
      throws IOException
  {
    final byte[] bytes = line.getBytes(StandardCharsets.US_ASCII);
    socket.send(new DatagramPacket(bytes, bytes.length, box, BOX_PORT));
  }

  private void onLine(@NonNull String[] fields)
  {
    if (!mRunning)
      return;
    final Esp32Protocol.Data data = Esp32Protocol.parseData(fields);
    if (data != null)
    {
      onData(data);
      return;
    }
    final Esp32Protocol.Reply reply = Esp32Protocol.parseReply(fields);
    if (reply != null)
    {
      onReply(reply);
      return;
    }
    final String imuName = Esp32Protocol.parseImuName(fields);
    if (imuName != null)
      mImuName = imuName;
  }

  private void onData(@NonNull Esp32Protocol.Data data)
  {
    final long now = SystemClock.elapsedRealtime();
    final boolean fresh = now - mLastDataMs <= DATA_TIMEOUT_MS;
    mLastDataMs = now;
    mFlags = data.flags;
    if (data.speedKmh >= 0)
      mListener.onSpeed(data.speedKmh, now - data.speedAgeMs);

    // The box sends the totals: a lost line loses nothing, the next one has the rotation.
    if (mHasLast && fresh)
    {
      final long dtMs = (data.timeMs - mLastTimeMs) & 0xFFFFFFFFL;
      final double yawDeltaDeg = isCalibrated() ? (data.yawMdeg - mLastYawMdeg) / 1000.0 : 0;
      mListener.onMotion(yawDeltaDeg, dtMs / 1000.0, SystemClock.elapsedRealtimeNanos());
    }
    mHasLast = true;
    mLastTimeMs = data.timeMs;
    mLastYawMdeg = data.yawMdeg;
  }

  private void onReply(@NonNull Esp32Protocol.Reply reply)
  {
    if (reply.id != mCalibrationId)
      return;
    if (reply.progress >= 0)
    {
      mCalibrationProgress = reply.progress;
      return;
    }
    Logger.i(TAG, "Calibration: ok = " + reply.ok + " error = " + reply.error);
    mCalibrationId = 0;
    mCalibrationFailed = !reply.ok;
  }

  private boolean isConnected()
  {
    return mRunning && mLastDataMs != 0 && SystemClock.elapsedRealtime() - mLastDataMs <= DATA_TIMEOUT_MS;
  }

  @NonNull
  @Override
  public State getState()
  {
    if (!mRunning)
      return State.DISCONNECTED;
    if (!isConnected())
      return State.CONNECTING;
    return (mFlags & Esp32Protocol.FLAG_OBD_OK) != 0 ? State.CONNECTED : State.NO_CAR_DATA;
  }

  @Override
  public boolean isCalibrated()
  {
    final int required = Esp32Protocol.FLAG_IMU_OK | Esp32Protocol.FLAG_BIAS_OK | Esp32Protocol.FLAG_UP_OK;
    return isConnected() && (mFlags & required) == required && (mFlags & Esp32Protocol.FLAG_MOUNT_MOVED) == 0;
  }

  @NonNull
  @Override
  public InertialNavigator.CalibrationState getCalibrationState()
  {
    if (mCalibrationId != 0 && SystemClock.elapsedRealtime() - mCalibrationStartMs > CALIBRATION_TIMEOUT_MS)
    {
      Logger.w(TAG, "No reply to the calibration");
      mCalibrationId = 0;
      mCalibrationFailed = true;
    }
    if (mCalibrationId != 0)
      return InertialNavigator.CalibrationState.CALIBRATING;
    if (mCalibrationFailed)
      return InertialNavigator.CalibrationState.FAILED_MOVING;
    if (isConnected() && (mFlags & Esp32Protocol.FLAG_MOUNT_MOVED) != 0)
      return InertialNavigator.CalibrationState.MOUNT_MOVED;
    return isCalibrated() ? InertialNavigator.CalibrationState.DONE : InertialNavigator.CalibrationState.NONE;
  }

  @Override
  public int getCalibrationProgressPercent()
  {
    return mCalibrationProgress;
  }

  @Override
  public void calibrate()
  {
    // The box zeroes the gyroscope and finds its vertical in the car at once.
    mCalibrationId = mNextCommandId++;
    mCalibrationStartMs = SystemClock.elapsedRealtime();
    mCalibrationProgress = 0;
    mCalibrationFailed = false;
    Logger.i(TAG, "Calibration, id = " + mCalibrationId);
    mOutgoing.add(Esp32Protocol.command(mCalibrationId, "CAL_UP"));
  }

  @Nullable
  @Override
  public String getDeviceName()
  {
    return mImuName != null ? "ESP32 · " + mImuName : "ESP32";
  }

  private static void sleep(long ms)
  {
    try
    {
      Thread.sleep(ms);
    }
    catch (InterruptedException e)
    {
      Thread.currentThread().interrupt();
    }
  }
}
