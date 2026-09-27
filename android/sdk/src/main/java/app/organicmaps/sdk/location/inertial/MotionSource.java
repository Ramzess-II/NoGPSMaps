package app.organicmaps.sdk.location.inertial;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * Where the inertial navigation takes the car movement from: the speed of the car and its rotation.
 */
public interface MotionSource
{
  enum State
  {
    DISCONNECTED,
    CONNECTING,
    // Connected, but the car doesn't tell its speed (ignition off, protocol search).
    NO_CAR_DATA,
    CONNECTED,
  }

  interface Listener
  {
    void onSpeed(int speedKmh, long elapsedRealtimeMs);

    /**
     * @param yawDeltaDeg the clockwise rotation of the car during dtSec, 0 while the source is not calibrated.
     * @param timestampNs elapsed realtime of the end of the interval.
     */
    void onMotion(double yawDeltaDeg, double dtSec, long timestampNs);
  }

  void start();

  void stop();

  @NonNull
  State getState();

  /**
   * @return true if the rotation of the car is known: the gyroscope is zeroed and its position in the car is
   * known.
   */
  boolean isCalibrated();

  @NonNull
  InertialNavigator.CalibrationState getCalibrationState();

  int getCalibrationProgressPercent();

  /**
   * Zeroes the gyroscope and finds where it looks in the car. The car must stand still for ~3 s.
   */
  void calibrate();

  /**
   * @return the name of the device to show, null for the phone.
   */
  @Nullable
  String getDeviceName();
}
