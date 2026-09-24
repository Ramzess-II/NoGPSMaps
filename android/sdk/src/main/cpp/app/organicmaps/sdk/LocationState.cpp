#include "Framework.hpp"
#include "map/gps_tracker.hpp"

#include "app/organicmaps/sdk/core/jni_helper.hpp"

#include "app/organicmaps/sdk/platform/AndroidPlatform.hpp"

#include "geometry/mercator.hpp"

extern "C"
{
static void LocationStateModeChanged(location::EMyPositionMode mode, std::shared_ptr<jobject> const & listener)
{
  JNIEnv * env = jni::GetEnv();
  env->CallVoidMethod(*listener, jni::GetMethodID(env, *listener.get(), "onMyPositionModeChanged", "(I)V"),
                      static_cast<jint>(mode));
}

//  public static void nativeSwitchToNextMode();
JNIEXPORT void Java_app_organicmaps_sdk_location_LocationState_nativeSwitchToNextMode(JNIEnv * env, jclass clazz)
{
  g_framework->SwitchMyPositionNextMode();
}

// private static int nativeGetMode();
JNIEXPORT jint Java_app_organicmaps_sdk_location_LocationState_nativeGetMode(JNIEnv * env, jclass clazz)
{
  // GetMyPositionMode() is initialized only after drape creation.
  // https://github.com/organicmaps/organicmaps/issues/1128#issuecomment-1784435190
  ASSERT(g_framework && g_framework->IsDrapeEngineCreated(), ());
  return g_framework->GetMyPositionMode();
}

//  public static void nativeSetListener(ModeChangeListener listener);
JNIEXPORT void Java_app_organicmaps_sdk_location_LocationState_nativeSetListener(JNIEnv * env, jclass clazz,
                                                                                 jobject listener)
{
  g_framework->SetMyPositionModeListener(
      std::bind(&LocationStateModeChanged, std::placeholders::_1, jni::make_global_ref(listener)));
}

//  public static void nativeRemoveListener();
JNIEXPORT void Java_app_organicmaps_sdk_location_LocationState_nativeRemoveListener(JNIEnv * env, jclass clazz)
{
  g_framework->SetMyPositionModeListener(location::TMyPositionModeChanged());
}

JNIEXPORT void Java_app_organicmaps_sdk_location_LocationState_nativeOnLocationError(JNIEnv * env, jclass clazz,
                                                                                     int errorCode)
{
  g_framework->OnLocationError(errorCode);
}

JNIEXPORT void Java_app_organicmaps_sdk_location_LocationState_nativeLocationUpdated(JNIEnv * env, jclass clazz,
                                                                                     jlong time, jdouble lat,
                                                                                     jdouble lon, jfloat accuracyH,
                                                                                     jdouble altitude, jfloat accuracyV,
                                                                                     jfloat speed, jfloat bearing)
{
  location::GpsInfo info;
  info.m_source = location::EAndroidNative;

  info.m_timestamp = static_cast<double>(time) / 1000.0;
  info.m_latitude = lat;
  info.m_longitude = lon;

  if (accuracyH > 0)
    info.m_horizontalAccuracy = accuracyH;

  if (accuracyV > 0)
  {
    info.m_altitude = altitude;
    info.m_verticalAccuracy = accuracyV;
  }

  if (bearing >= 0)
    info.m_bearing = bearing;

  if (speed >= 0)
    info.m_speed = speed;

  g_framework->OnLocationUpdated(info);
  GpsTracker::Instance().OnLocationUpdated(info);
}

// public static native double[] nativeScreenToLatLon(float x, float y);
JNIEXPORT jdoubleArray Java_app_organicmaps_sdk_location_LocationState_nativeScreenToLatLon(JNIEnv * env,
                                                                                            jclass clazz, jfloat x,
                                                                                            jfloat y)
{
  // P3dtoG takes into account the perspective view used in navigation mode.
  auto const ll = mercator::ToLatLon(g_framework->NativeFramework()->P3dtoG(m2::PointD(x, y)));
  jdouble const coords[] = {ll.m_lat, ll.m_lon};
  jdoubleArray result = env->NewDoubleArray(2);
  env->SetDoubleArrayRegion(result, 0, 2, coords);
  return result;
}

// public static native void nativeRebuildRouteIfOffRoute(long time, double lat, double lon, float accuracy);
JNIEXPORT void Java_app_organicmaps_sdk_location_LocationState_nativeRebuildRouteIfOffRoute(JNIEnv * env,
                                                                                            jclass clazz, jlong time,
                                                                                            jdouble lat, jdouble lon,
                                                                                            jfloat accuracy)
{
  location::GpsInfo info;
  info.m_source = location::EUser;
  info.m_timestamp = static_cast<double>(time) / 1000.0;
  info.m_latitude = lat;
  info.m_longitude = lon;
  info.m_horizontalAccuracy = accuracy;
  g_framework->NativeFramework()->GetRoutingManager().RebuildRouteIfOffRoute(info);
}

// public static native double[] nativeSnapToRoad(double lat, double lon, double bearing, double radius,
//                                                 boolean matchRoute);
JNIEXPORT jdoubleArray Java_app_organicmaps_sdk_location_LocationState_nativeSnapToRoad(JNIEnv * env, jclass clazz,
                                                                                        jdouble lat, jdouble lon,
                                                                                        jdouble bearing, jdouble radius,
                                                                                        jboolean matchRoute)
{
  ms::LatLon snapped;
  double snappedBearing;
  if (!g_framework->NativeFramework()->GetRoutingManager().SnapToRoad(ms::LatLon(lat, lon), bearing, radius, matchRoute,
                                                                      snapped, snappedBearing))
  {
    return nullptr;
  }
  jdouble const values[] = {snapped.m_lat, snapped.m_lon, snappedBearing};
  jdoubleArray result = env->NewDoubleArray(3);
  env->SetDoubleArrayRegion(result, 0, 3, values);
  return result;
}

// public static native double[] nativeProjectToRoute(double lat, double lon, double radius);
JNIEXPORT jdoubleArray Java_app_organicmaps_sdk_location_LocationState_nativeProjectToRoute(JNIEnv * env, jclass clazz,
                                                                                            jdouble lat, jdouble lon,
                                                                                            jdouble radius)
{
  ms::LatLon projected;
  double bearing;
  if (!g_framework->NativeFramework()->GetRoutingManager().ProjectToRoute(ms::LatLon(lat, lon), radius, projected,
                                                                          bearing))
  {
    return nullptr;
  }
  jdouble const values[] = {projected.m_lat, projected.m_lon, bearing};
  jdoubleArray result = env->NewDoubleArray(3);
  env->SetDoubleArrayRegion(result, 0, 3, values);
  return result;
}

// public static native double[] nativeShiftAlongRoute(double lat, double lon, double bearing, double distance);
JNIEXPORT jdoubleArray Java_app_organicmaps_sdk_location_LocationState_nativeShiftAlongRoute(
    JNIEnv * env, jclass clazz, jdouble lat, jdouble lon, jdouble bearing, jdouble distance)
{
  ms::LatLon shifted;
  double shiftedBearing;
  double applied;
  bool atCrossing;
  if (!g_framework->NativeFramework()->GetRoutingManager().ShiftAlongRoute(ms::LatLon(lat, lon), bearing, distance,
                                                                           shifted, shiftedBearing, applied,
                                                                           atCrossing))
  {
    return nullptr;
  }

  jdouble const values[] = {shifted.m_lat, shifted.m_lon, shiftedBearing, applied, atCrossing ? 1.0 : 0.0};
  jdoubleArray result = env->NewDoubleArray(5);
  env->SetDoubleArrayRegion(result, 0, 5, values);
  return result;
}
}  // extern "C"
