package app.organicmaps.sdk.location;

import android.content.Context;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.UiThread;
import app.organicmaps.sdk.util.log.Logger;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.util.Arrays;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * The UDP connection to the NoGPS ESP32 sensor box for the navigation without GPS in the core: the core talks to the
 * box, this class only sends its lines and passes the received datagrams back on the UI thread.
 */
class Esp32Transport
{
  private static final String TAG = Esp32Transport.class.getSimpleName();
  // The box sends 50 lines a second, the queued lines are sent between them.
  private static final int RECEIVE_TIMEOUT_MS = 200;
  private static final long RETRY_DELAY_MS = 1000;

  @Nullable
  private final ConnectivityManager mConnectivityManager;
  private final Handler mMainHandler = new Handler(Looper.getMainLooper());
  private final Queue<byte[]> mOutgoing = new ConcurrentLinkedQueue<>();
  // The thread talking to the box, a closed one ends by itself.
  @Nullable
  private volatile Thread mThread;
  // The Wi-Fi network of the box. It has no internet, so Android sends everything else over the mobile data,
  // and the sockets to the box must be bound to it.
  @Nullable
  private volatile Network mWifi;
  @Nullable
  private ConnectivityManager.NetworkCallback mNetworkCallback;

  Esp32Transport(@NonNull Context context)
  {
    mConnectivityManager = (ConnectivityManager) context.getSystemService(Context.CONNECTIVITY_SERVICE);
  }

  @UiThread
  void open(@NonNull String host, int port)
  {
    close();
    Logger.i(TAG, "address = " + host + ":" + port);
    watchWifi();
    final Thread thread = new Thread(() -> run(host, port), "Esp32");
    mThread = thread;
    thread.start();
  }

  @UiThread
  void send(@NonNull byte[] line)
  {
    if (mThread != null)
      mOutgoing.add(line);
  }

  @UiThread
  void close()
  {
    final Thread thread = mThread;
    mThread = null;
    if (thread != null)
      thread.interrupt();
    mOutgoing.clear();
    if (mConnectivityManager != null && mNetworkCallback != null)
      mConnectivityManager.unregisterNetworkCallback(mNetworkCallback);
    mNetworkCallback = null;
    mWifi = null;
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
    return Thread.currentThread() == mThread;
  }

  private void run(@NonNull String host, int port)
  {
    while (isCurrentThread())
    {
      final Network network = mWifi;
      try (DatagramSocket socket = new DatagramSocket())
      {
        if (network != null && Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP_MR1)
          network.bindSocket(socket);
        socket.setSoTimeout(RECEIVE_TIMEOUT_MS);
        exchange(socket, network, InetAddress.getByName(host), port);
      }
      catch (IOException e)
      {
        Logger.w(TAG, "Socket error", e);
        try
        {
          Thread.sleep(RETRY_DELAY_MS);
        }
        catch (InterruptedException ie)
        {
          return;
        }
      }
    }
  }

  /**
   * Talks to the box until the Wi-Fi network changes or the transport is closed.
   */
  private void exchange(@NonNull DatagramSocket socket, @Nullable Network network, @NonNull InetAddress box, int port)
      throws IOException
  {
    final byte[] buffer = new byte[512];
    while (isCurrentThread() && network == mWifi)
    {
      for (byte[] line = mOutgoing.poll(); line != null; line = mOutgoing.poll())
        socket.send(new DatagramPacket(line, line.length, box, port));

      final DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
      try
      {
        socket.receive(packet);
      }
      catch (SocketTimeoutException e)
      {
        continue;
      }
      final byte[] data = Arrays.copyOf(packet.getData(), packet.getLength());
      final Thread thread = Thread.currentThread();
      mMainHandler.post(() -> {
        // The datagrams of a closed connection are dropped.
        if (thread == mThread)
          NoGps.nativeOnEsp32Datagram(data);
      });
    }
  }
}
