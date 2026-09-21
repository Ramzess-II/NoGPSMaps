package app.organicmaps.sdk;

import android.app.Activity;
import android.content.Context;
import android.content.ContextWrapper;
import android.content.res.TypedArray;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.ViewConfiguration;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.content.res.ConfigurationHelper;
import app.organicmaps.sdk.display.DisplayType;
import app.organicmaps.sdk.util.Utils;
import app.organicmaps.sdk.util.log.Logger;

public class MapView extends SurfaceView
{
  private static final String TAG = MapView.class.getSimpleName();

  public interface TapInterceptor
  {
    void onMapTap(float x, float y);
  }

  // When set, short taps on the map go to the interceptor instead of the core (e.g. to set own position manually).
  @Nullable
  private static TapInterceptor sTapInterceptor;

  private boolean mTapCandidate;
  private float mTapDownX;
  private float mTapDownY;
  private long mTapDownTime;

  private class SurfaceHolderCallback implements SurfaceHolder.Callback
  {
    @Override
    public void surfaceCreated(@NonNull SurfaceHolder holder)
    {
      Logger.d(TAG);
      mMap.onSurfaceCreated(MapView.this.getContext(), holder.getSurface(), holder.getSurfaceFrame(),
                            ConfigurationHelper.getDensityDpi(MapView.this.getResources()));
    }

    @Override
    public void surfaceChanged(@NonNull SurfaceHolder holder, int format, int width, int height)
    {
      Logger.d(TAG);
      mMap.onSurfaceChanged(MapView.this.getContext(), holder.getSurface(), holder.getSurfaceFrame(),
                            holder.isCreating());
    }

    @Override
    public void surfaceDestroyed(@NonNull SurfaceHolder holder)
    {
      Logger.d(TAG);
      mMap.onSurfaceDestroyed(isHostActivityChangingConfigurations());
    }
  }

  @NonNull
  private final Map mMap;

  public MapView(@NonNull Context context)
  {
    this(context, null, 0);
  }

  public MapView(@NonNull Context context, @Nullable AttributeSet attrs)
  {
    this(context, attrs, 0);
  }

  public MapView(@NonNull Context context, @Nullable AttributeSet attrs, int defStyleAttr)
  {
    this(context, attrs, defStyleAttr, R.style.MapView);
  }

  public MapView(@NonNull Context context, @Nullable AttributeSet attrs, int defStyleAttr, int defStyleRes)
  {
    super(context, attrs, defStyleAttr, defStyleRes);

    try (final TypedArray data =
             context.getTheme().obtainStyledAttributes(attrs, R.styleable.MapView, defStyleAttr, defStyleRes))
    {
      final int displayTypeOrdinal = data.getInt(R.styleable.MapView_display_type, DisplayType.Device.ordinal());
      final DisplayType displayType = DisplayType.values()[displayTypeOrdinal];
      mMap = new Map(displayType);
    }
    getHolder().addCallback(new SurfaceHolderCallback());
  }

  public final void onDraw(@NonNull Canvas canvas)
  {
    super.onDraw(canvas);
    if (isInEditMode())
      drawMapPreview(canvas);
  }

  @Override
  public boolean onTouchEvent(@NonNull MotionEvent event)
  {
    int action = event.getActionMasked();
    int pointerIndex = event.getActionIndex();
    switch (action)
    {
    case MotionEvent.ACTION_POINTER_UP -> action = Map.NATIVE_ACTION_UP;
    case MotionEvent.ACTION_UP ->
    {
      action = Map.NATIVE_ACTION_UP;
      pointerIndex = 0;
    }
    case MotionEvent.ACTION_POINTER_DOWN -> action = Map.NATIVE_ACTION_DOWN;
    case MotionEvent.ACTION_DOWN ->
    {
      action = Map.NATIVE_ACTION_DOWN;
      pointerIndex = 0;
    }
    case MotionEvent.ACTION_MOVE ->
    {
      action = Map.NATIVE_ACTION_MOVE;
      pointerIndex = Map.INVALID_POINTER_MASK;
    }
    case MotionEvent.ACTION_CANCEL -> action = Map.NATIVE_ACTION_CANCEL;
    }

    if (isInterceptedTap(event))
    {
      // Cancel the gesture in the core, so it doesn't select an object under the tap.
      Map.onTouch(Map.NATIVE_ACTION_CANCEL, event, pointerIndex);
      if (sTapInterceptor != null)
        sTapInterceptor.onMapTap(event.getX(), event.getY());
      return true;
    }

    Map.onTouch(action, event, pointerIndex);
    performClick();
    return true;
  }

  public static void setTapInterceptor(@Nullable TapInterceptor interceptor)
  {
    sTapInterceptor = interceptor;
  }

  private boolean isInterceptedTap(@NonNull MotionEvent event)
  {
    switch (event.getActionMasked())
    {
    case MotionEvent.ACTION_DOWN ->
    {
      mTapCandidate = sTapInterceptor != null;
      mTapDownX = event.getX();
      mTapDownY = event.getY();
      mTapDownTime = event.getEventTime();
    }
    case MotionEvent.ACTION_POINTER_DOWN, MotionEvent.ACTION_CANCEL -> mTapCandidate = false;
    case MotionEvent.ACTION_MOVE ->
    {
      final int slop = ViewConfiguration.get(getContext()).getScaledTouchSlop();
      if (Math.hypot(event.getX() - mTapDownX, event.getY() - mTapDownY) > slop)
        mTapCandidate = false;
    }
    case MotionEvent.ACTION_UP ->
    {
      final boolean isTap = mTapCandidate && sTapInterceptor != null
                         && event.getEventTime() - mTapDownTime < ViewConfiguration.getLongPressTimeout();
      mTapCandidate = false;
      return isTap;
    }
    }
    return false;
  }

  @Override
  public boolean performClick()
  {
    super.performClick();
    return false;
  }

  @NonNull
  Map getMap()
  {
    return mMap;
  }

  ///  The function is called only in the design mode of Android Studio.
  private void drawMapPreview(@NonNull Canvas canvas)
  {
    final int w = getWidth();
    final int h = getHeight();

    // Background
    final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    paint.setStyle(Paint.Style.FILL);
    if (Utils.isDarkMode(getContext()))
      paint.setColor(Color.rgb(30, 30, 30));
    else
      paint.setColor(Color.rgb(245, 242, 230));
    canvas.drawRect(0, 0, w, h, paint);

    // Grid lines (lat/lon)
    paint.setColor(Color.LTGRAY);
    paint.setStrokeWidth(2f);
    final int step = Math.min(w, h) / 6;
    for (int i = 0; i < Math.max(w, h); i += step)
    {
      if (i < w)
        canvas.drawLine(i, 0, i, h, paint);
      if (i < h)
        canvas.drawLine(0, i, w, i, paint);
    }
  }

  private boolean isHostActivityChangingConfigurations()
  {
    Activity activity = findActivity(getContext());
    return activity != null && activity.isChangingConfigurations();
  }

  private static Activity findActivity(Context context)
  {
    while (context instanceof ContextWrapper)
    {
      if (context instanceof Activity)
      {
        return (Activity) context;
      }
      context = ((ContextWrapper) context).getBaseContext();
    }
    return null;
  }
}
