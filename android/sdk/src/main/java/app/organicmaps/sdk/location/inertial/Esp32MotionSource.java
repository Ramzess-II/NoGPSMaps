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
import java.util.TreeSet;
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
  // Events lost over Wi-Fi are asked again not more often than this.
  private static final long EVENTS_REQUEST_INTERVAL_MS = 2000;
  // More than the box keeps in its journal.
  private static final int MAX_LOGGED_EVENTS = 64;

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
  // The state of the ELM327 adapter of the box from its status, null until it comes.
  @Nullable
  private String mObdState;
  // The box hasn't found its ELM327: while it searches it again, the adapter is still missing.
  private boolean mObdAdapterMissing;
  // The engine, its speed and the voltage of the car from the status of the box, null until it comes.
  @Nullable
  private CarInfo mCarInfo;
  // The box has told that its voltage and the one of ELM327 differ. Kept until they come close again.
  private boolean mVoltageMismatch;
  // The numbers of the events of the box written to the log.
  private final TreeSet<Long> mLoggedEvents = new TreeSet<>();
  private long mEventsRequestMs;
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
    final Esp32Protocol.Event event = Esp32Protocol.parseEvent(fields);
    if (event != null)
    {
      onEvent(event);
      return;
    }
    onLastEventNumber(Esp32Protocol.parseLastEventNumber(fields));
    final String imuName = Esp32Protocol.parseImuName(fields);
    if (imuName != null)
      mImuName = imuName.isEmpty() ? null : imuName;
    final CarInfo carInfo = Esp32Protocol.parseCarInfo(fields);
    if (carInfo != null)
    {
      if (carInfo.boxMillivolts >= 0 && carInfo.getReferenceMillivolts() >= 0)
        mVoltageMismatch = carInfo.voltageMismatch;
      mCarInfo = carInfo;
    }
    final String obdState = Esp32Protocol.parseObdState(fields);
    if (obdState != null && !obdState.equals(mObdState))
    {
      Logger.i(TAG, "OBD state = " + obdState);
      mObdState = obdState;
      if ("NO_ADAPTER".equals(obdState))
        mObdAdapterMissing = true;
      else if (!"INIT".equals(obdState))
        mObdAdapterMissing = false;
    }
  }

  private final AccelLog mAccelLog = new AccelLog();

  private void onData(@NonNull Esp32Protocol.Data data)
  {
    final long now = SystemClock.elapsedRealtime();
    final boolean fresh = now - mLastDataMs <= DATA_TIMEOUT_MS;
    mLastDataMs = now;
    mFlags = data.flags;
    if (data.speedKmh >= 0)
      mListener.onSpeed(data.speedKmh, now - data.speedAgeMs);
    // For the trip log only, see AccelLog.
    if (data.hasAccel)
    {
      final String line =
          mAccelLog.onSample(data.timeMs * 1_000_000, data.accelH1, data.accelH2, data.accelUp, data.jolt);
      if (line != null && data.speedKmh > 0)
        Logger.i(TAG, "ACCB " + line);
    }

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

  /**
   * Writes the errors of the box to the log, to find later why the speed or the gyroscope was lost.
   */
  private void onEvent(@NonNull Esp32Protocol.Event event)
  {
    final long lastNumber = mLoggedEvents.isEmpty() ? -1 : mLoggedEvents.last();
    // The journal of the box sent on request comes after the new events, so the order doesn't matter.
    if (!mLoggedEvents.add(event.number))
      return;
    if (mLoggedEvents.size() > MAX_LOGGED_EVENTS)
      mLoggedEvents.pollFirst();
    final String message = "Box event #" + event.number + " at " + event.timeMs + " ms: " + event.code + " "
                         + event.text;
    if ("I".equals(event.level))
      Logger.i(TAG, message);
    else
      Logger.w(TAG, message);
    // The box compares the raw voltage, the status has the corrected one.
    if ("VOLT_MISMATCH".equals(event.code))
      mVoltageMismatch = true;
    else if ("VOLT_CAL".equals(event.code))
      mVoltageMismatch = false;
    if (event.number > lastNumber + 1)
      requestLostEvents(lastNumber + 1);
  }

  /**
   * Asks the journal of the box for the events since the given number, which were lost over Wi-Fi or came before
   * the connection.
   */
  private void requestLostEvents(long fromNumber)
  {
    final long now = SystemClock.elapsedRealtime();
    if (now - mEventsRequestMs < EVENTS_REQUEST_INTERVAL_MS)
      return;
    mEventsRequestMs = now;
    mOutgoing.add(Esp32Protocol.command(mNextCommandId++, "EVENTS," + fromNumber));
  }

  private void onLastEventNumber(long lastNumber)
  {
    if (lastNumber < 0)
      return;
    // The box has restarted and counts its events again.
    if (!mLoggedEvents.isEmpty() && lastNumber < mLoggedEvents.last())
      mLoggedEvents.clear();
    final long loggedNumber = mLoggedEvents.isEmpty() ? -1 : mLoggedEvents.last();
    if (lastNumber > loggedNumber)
      requestLostEvents(loggedNumber + 1);
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
      return isSleeping() ? State.BOX_SLEEPING : State.CONNECTING;
    if ((mFlags & Esp32Protocol.FLAG_OBD_OK) != 0)
      return State.CONNECTED;
    if ("SLEEP".equals(mObdState))
      return State.OBD_SLEEPING;
    // OBD_ABSENT is set also when ELM327 is off in the box, only the status tells one from another.
    if (mObdAdapterMissing)
      return State.NO_ADAPTER;
    if (mObdState == null)
      return State.NO_CAR_DATA;
    return switch (mObdState)
    {
      case "DISABLED" -> State.OBD_DISABLED;
      case "INIT", "SEARCHING" -> State.OBD_CONNECTING;
      case "ERROR" -> State.OBD_ERROR;
      // NO_CAR and NO_DATA: the adapter answers, the car doesn't.
      default -> State.NO_CAR_DATA;
    };
  }

  /**
   * The box switches its Wi-Fi off a minute after the engine is stopped, so a box that has gone after it has
   * told about the stopped engine is not lost.
   */
  private boolean isSleeping()
  {
    if (mLastDataMs == 0)
      return false;
    return "SLEEP".equals(mObdState) || (mCarInfo != null && Boolean.FALSE.equals(mCarInfo.engineRunning));
  }

  @Nullable
  @Override
  public CarInfo getCarInfo()
  {
    if (!isConnected() || mCarInfo == null)
      return null;
    return new CarInfo(mCarInfo.engineRunning, mCarInfo.rpm, mCarInfo.boxMillivolts, mCarInfo.elmMillivolts,
                       mCarInfo.ecuMillivolts, mVoltageMismatch);
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
