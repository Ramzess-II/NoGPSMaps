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
    // The ELM327 adapter doesn't answer: it is not plugged in, switched off or broken.
    NO_ADAPTER,
    // The sensor box is set up not to use ELM327.
    OBD_DISABLED,
    // The sensor box is connected, its ELM327 is being initialized or searches the protocol of the car.
    OBD_CONNECTING,
    // ELM327 of the sensor box has a bus error or has stopped answering.
    OBD_ERROR,
    // Connected, but the car doesn't tell its speed (ignition off, protocol search).
    NO_CAR_DATA,
    // The sensor box has gone after the engine was stopped: it switches its Wi-Fi off to save the car battery.
    BOX_SLEEPING,
    // The sensor box is connected and doesn't talk to the car with the engine stopped.
    OBD_SLEEPING,
    CONNECTED,
  }

  /**
   * What the sensor box knows about the car besides its speed. The box asks the car for the engine speed and
   * ELM327 for the voltage only while the car stands, not to delay the speed.
   */
  final class CarInfo
  {
    // null if unknown.
    @Nullable
    public final Boolean engineRunning;
    // -1 if unknown.
    public final int rpm;
    // The voltage of the car measured by the box itself and by ELM327, -1 if unknown.
    public final int boxMillivolts;
    public final int elmMillivolts;
    // The two voltages differ too much: the voltage divider of the box is wrong.
    public final boolean voltageMismatch;

    public CarInfo(@Nullable Boolean engineRunning, int rpm, int boxMillivolts, int elmMillivolts,
                   boolean voltageMismatch)
    {
      this.engineRunning = engineRunning;
      this.rpm = rpm;
      this.boxMillivolts = boxMillivolts;
      this.elmMillivolts = elmMillivolts;
      this.voltageMismatch = voltageMismatch;
    }
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

  /**
   * @return null if the source doesn't tell it: the phone, or the box that is not connected.
   */
  @Nullable
  default CarInfo getCarInfo()
  {
    return null;
  }
}
