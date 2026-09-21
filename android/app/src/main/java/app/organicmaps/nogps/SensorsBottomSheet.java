package app.organicmaps.nogps;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;
import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.widget.SwitchCompat;
import androidx.core.content.ContextCompat;
import app.organicmaps.MwmActivity;
import app.organicmaps.MwmApplication;
import app.organicmaps.R;
import app.organicmaps.sdk.location.LocationHelper;
import app.organicmaps.sdk.location.inertial.Elm327Client;
import app.organicmaps.sdk.location.inertial.InertialNavigator;
import app.organicmaps.sdk.util.Config;
import com.google.android.material.bottomsheet.BottomSheetDialogFragment;
import com.google.android.material.dialog.MaterialAlertDialogBuilder;
import java.util.ArrayList;
import java.util.List;

/**
 * Inertial navigation settings and state: the ELM327 adapter, speed, gyroscope zeroing and the car direction.
 */
public class SensorsBottomSheet extends BottomSheetDialogFragment
{
  public static final String TAG = SensorsBottomSheet.class.getSimpleName();
  private static final long UPDATE_INTERVAL_MS = 500;

  private final Handler mHandler = new Handler(Looper.getMainLooper());
  private final Runnable mUpdater = new Runnable() {
    @Override
    public void run()
    {
      update();
      mHandler.postDelayed(this, UPDATE_INTERVAL_MS);
    }
  };

  private final ActivityResultLauncher<String> mBluetoothPermissionLauncher =
      registerForActivityResult(new ActivityResultContracts.RequestPermission(), this::onBluetoothPermissionResult);
  // What to do after the Bluetooth permission is granted.
  @Nullable
  private Runnable mAfterPermission;

  private SwitchCompat mSwitch;
  private TextView mAdapter;
  private TextView mSpeed;
  private TextView mGyro;
  private TextView mHeading;
  private TextView mReadiness;
  private View[] mRotateButtons;

  @Nullable
  @Override
  public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container,
                           @Nullable Bundle savedInstanceState)
  {
    final View view = inflater.inflate(R.layout.nogps_sensors_sheet, container, false);
    mSwitch = view.findViewById(R.id.nogps_inertial_switch);
    mAdapter = view.findViewById(R.id.nogps_adapter);
    mSpeed = view.findViewById(R.id.nogps_speed);
    mGyro = view.findViewById(R.id.nogps_gyro);
    mHeading = view.findViewById(R.id.nogps_heading);
    mReadiness = view.findViewById(R.id.nogps_readiness);

    mSwitch.setChecked(getLocationHelper().isInertialNavigationEnabled());
    mSwitch.setOnCheckedChangeListener((v, isChecked) -> onSwitch(isChecked));
    view.findViewById(R.id.nogps_choose_adapter).setOnClickListener(v -> withBluetoothPermission(this::chooseAdapter));
    view.findViewById(R.id.nogps_calibrate).setOnClickListener(v -> {
      final InertialNavigator inertial = getLocationHelper().getInertialNavigator();
      if (inertial != null)
        inertial.calibrate();
    });
    mRotateButtons = new View[] {view.findViewById(R.id.nogps_rotate_left_45),
                                 view.findViewById(R.id.nogps_rotate_left_10),
                                 view.findViewById(R.id.nogps_rotate_right_10),
                                 view.findViewById(R.id.nogps_rotate_right_45)};
    final int[] rotations = {-45, -10, 10, 45};
    for (int i = 0; i < mRotateButtons.length; i++)
    {
      final int rotation = rotations[i];
      mRotateButtons[i].setOnClickListener(v -> {
        getLocationHelper().rotateHeading(rotation);
        update();
      });
    }
    view.findViewById(R.id.nogps_set_heading).setOnClickListener(v -> {
      ((MwmActivity) requireActivity()).startHeadingSelection();
      dismiss();
    });
    return view;
  }

  @Override
  public void onStart()
  {
    super.onStart();
    mHandler.post(mUpdater);
  }

  @Override
  public void onStop()
  {
    super.onStop();
    mHandler.removeCallbacks(mUpdater);
  }

  @NonNull
  private LocationHelper getLocationHelper()
  {
    return MwmApplication.from(requireContext()).getLocationHelper();
  }

  private void onSwitch(boolean enabled)
  {
    if (!enabled)
    {
      getLocationHelper().setInertialNavigationEnabled(false);
      update();
      return;
    }
    withBluetoothPermission(() -> {
      getLocationHelper().setInertialNavigationEnabled(true);
      if (Config.getElm327Address() == null)
        chooseAdapter();
      update();
    });
  }

  private void withBluetoothPermission(@NonNull Runnable action)
  {
    if (Build.VERSION.SDK_INT < Build.VERSION_CODES.S
        || ContextCompat.checkSelfPermission(requireContext(), Manifest.permission.BLUETOOTH_CONNECT)
               == PackageManager.PERMISSION_GRANTED)
    {
      action.run();
      return;
    }
    mAfterPermission = action;
    mBluetoothPermissionLauncher.launch(Manifest.permission.BLUETOOTH_CONNECT);
  }

  private void onBluetoothPermissionResult(boolean granted)
  {
    final Runnable action = mAfterPermission;
    mAfterPermission = null;
    if (granted && action != null)
    {
      action.run();
      return;
    }
    Toast.makeText(requireContext(), R.string.nogps_bt_permission_denied, Toast.LENGTH_LONG).show();
    mSwitch.setChecked(getLocationHelper().isInertialNavigationEnabled());
  }

  @SuppressLint("MissingPermission") // Checked in withBluetoothPermission().
  private void chooseAdapter()
  {
    final BluetoothAdapter bluetooth = BluetoothAdapter.getDefaultAdapter();
    final List<BluetoothDevice> devices =
        bluetooth == null ? new ArrayList<>() : new ArrayList<>(bluetooth.getBondedDevices());
    if (devices.isEmpty())
    {
      Toast.makeText(requireContext(), R.string.nogps_no_paired, Toast.LENGTH_LONG).show();
      return;
    }
    final String[] names = new String[devices.size()];
    for (int i = 0; i < devices.size(); i++)
    {
      final String name = devices.get(i).getName();
      names[i] = (name != null ? name : "?") + "  (" + devices.get(i).getAddress() + ")";
    }
    new MaterialAlertDialogBuilder(requireContext())
        .setTitle(R.string.nogps_choose_adapter)
        .setItems(names, (dialog, which) -> getLocationHelper().setElm327Address(devices.get(which).getAddress()))
        .show();
  }

  @SuppressLint("MissingPermission")
  @NonNull
  private String adapterName()
  {
    final String address = Config.getElm327Address();
    if (address == null)
      return getString(R.string.nogps_sensors_adapter_none);
    final BluetoothAdapter bluetooth = BluetoothAdapter.getDefaultAdapter();
    try
    {
      if (bluetooth != null)
      {
        final String name = bluetooth.getRemoteDevice(address).getName();
        if (name != null)
          return name;
      }
    }
    catch (SecurityException e)
    {
      // No Bluetooth permission, show the address.
    }
    return address;
  }

  private void update()
  {
    if (getContext() == null)
      return;
    final LocationHelper locationHelper = getLocationHelper();
    final InertialNavigator inertial = locationHelper.getInertialNavigator();
    final boolean enabled = locationHelper.isInertialNavigationEnabled() && inertial != null;

    String adapter = adapterName();
    if (enabled)
      adapter += " · " + getString(elmStateText(inertial.getElm327State()));
    mAdapter.setText(getString(R.string.nogps_sensors_adapter, adapter));

    final int speed = enabled ? inertial.getSpeedKmh() : -1;
    mSpeed.setText(getString(R.string.nogps_sensors_speed, speed >= 0 ? getString(R.string.nogps_speed_kmh, speed)
                                                                       : getString(R.string.nogps_unknown)));

    final InertialNavigator.CalibrationState calibration =
        enabled ? inertial.getCalibrationState() : InertialNavigator.CalibrationState.NONE;
    final String gyro = switch (calibration)
    {
      case NONE -> getString(R.string.nogps_gyro_none);
      case CALIBRATING -> getString(R.string.nogps_gyro_calibrating, inertial.getCalibrationProgressPercent());
      case DONE -> getString(R.string.nogps_gyro_done);
      case FAILED_MOVING -> getString(R.string.nogps_gyro_failed);
    };
    mGyro.setText(getString(R.string.nogps_sensors_gyro, gyro));

    final InertialNavigator.HeadingSource heading =
        enabled ? inertial.getHeadingSource() : InertialNavigator.HeadingSource.NONE;
    final int headingText = switch (heading)
    {
      case NONE -> R.string.nogps_heading_none;
      case GPS -> R.string.nogps_heading_gps;
      case MANUAL_MARKS -> R.string.nogps_heading_marks;
      case USER -> R.string.nogps_heading_user;
    };
    final double headingDeg = enabled ? inertial.getHeading() : Double.NaN;
    final String headingValue = Double.isNaN(headingDeg)
                                  ? getString(headingText)
                                  : Math.round(headingDeg) + "° · " + getString(headingText);
    mHeading.setText(getString(R.string.nogps_sensors_heading, headingValue));
    for (View button : mRotateButtons)
      button.setEnabled(!Double.isNaN(headingDeg));

    if (!enabled)
    {
      mReadiness.setText("");
      return;
    }
    final List<String> missing = new ArrayList<>();
    if (speed < 0)
      missing.add(getString(R.string.nogps_missing_speed));
    if (calibration != InertialNavigator.CalibrationState.DONE)
      missing.add(getString(R.string.nogps_missing_gyro));
    if (!inertial.hasPosition())
      missing.add(getString(R.string.nogps_missing_position));
    if (heading == InertialNavigator.HeadingSource.NONE)
      missing.add(getString(R.string.nogps_missing_heading));
    if (missing.isEmpty())
    {
      mReadiness.setText(R.string.nogps_sensors_ready);
      mReadiness.setTextColor(ContextCompat.getColor(requireContext(), R.color.nogps_status_gps));
    }
    else
    {
      mReadiness.setText(getString(R.string.nogps_sensors_not_ready, String.join(", ", missing)));
      mReadiness.setTextColor(ContextCompat.getColor(requireContext(), R.color.nogps_status_none));
    }
  }

  private static int elmStateText(@NonNull Elm327Client.State state)
  {
    return switch (state)
    {
      case DISCONNECTED -> R.string.nogps_elm_disconnected;
      case CONNECTING -> R.string.nogps_elm_connecting;
      case NO_CAR_DATA -> R.string.nogps_elm_no_car;
      case CONNECTED -> R.string.nogps_elm_connected;
    };
  }
}
