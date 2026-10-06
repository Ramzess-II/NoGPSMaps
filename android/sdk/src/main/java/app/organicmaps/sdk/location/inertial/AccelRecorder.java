package app.organicmaps.sdk.location.inertial;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import androidx.annotation.NonNull;
import app.organicmaps.sdk.util.log.Logger;

/**
 * Writes the accelerometer of the phone to the trip log while the car moves, see AccelLog. Nothing depends
 * on it yet.
 */
public final class AccelRecorder implements SensorEventListener
{
  private static final String TAG = AccelRecorder.class.getSimpleName();

  private final SensorManager mSensorManager;
  private final AccelLog mLog = new AccelLog();
  private boolean mMoving;

  public AccelRecorder(@NonNull Context context)
  {
    mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
  }

  public void start()
  {
    final Sensor accel = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    if (accel != null)
      mSensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_GAME);
  }

  public void stop()
  {
    mSensorManager.unregisterListener(this);
    mLog.reset();
  }

  public void setMoving(boolean moving)
  {
    mMoving = moving;
  }

  @Override
  public void onSensorChanged(SensorEvent event)
  {
    final String line = mLog.onSample(event.timestamp, event.values[0], event.values[1], event.values[2]);
    if (line != null && mMoving)
      Logger.i(TAG, "ACC " + line);
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int accuracy)
  {}
}
