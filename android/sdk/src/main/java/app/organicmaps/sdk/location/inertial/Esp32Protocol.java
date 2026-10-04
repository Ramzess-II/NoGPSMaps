package app.organicmaps.sdk.location.inertial;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * Lines of the NoGPS ESP32 sensor box protocol, version 1: "$NGD,...*CRC\r\n". The checksum is
 * CRC-16/CCITT-FALSE of the bytes between '$' and '*'.
 */
public final class Esp32Protocol
{
  static final int VERSION = 1;

  // Bits of the data flags.
  static final int FLAG_IMU_OK = 1;
  static final int FLAG_BIAS_OK = 1 << 1;
  static final int FLAG_UP_OK = 1 << 2;
  static final int FLAG_FWD_OK = 1 << 3;
  static final int FLAG_OBD_OK = 1 << 4;
  static final int FLAG_STILL = 1 << 5;
  static final int FLAG_CALIBRATING = 1 << 6;
  static final int FLAG_MOUNT_MOVED = 1 << 7;
  static final int FLAG_REVERSE = 1 << 8;
  static final int FLAG_OBD_ABSENT = 1 << 9;
  static final int FLAG_ERROR = 1 << 10;
  // The box tells about a wrong voltage divider from the same difference.
  static final int MAX_VOLTAGE_DIFF_MV = 1000;

  /**
   * A data line: the totals since the box has started, so a lost line loses nothing.
   */
  public static final class Data
  {
    public long seq;
    public long timeMs;
    // Clockwise, thousandths of a degree.
    public long yawMdeg;
    public long rateMdps;
    public long distMm;
    // -1 if the car doesn't tell it.
    public int speedKmh;
    public int speedAgeMs;
    public int flags;
  }

  /**
   * An event of the box: an error, its end or a calibration.
   */
  public static final class Event
  {
    public long number;
    public long timeMs;
    // E - error, W - warning, I - information.
    @NonNull
    public String level = "";
    @NonNull
    public String code = "";
    @NonNull
    public String text = "";
  }

  /**
   * A reply to a command.
   */
  public static final class Reply
  {
    public int id;
    public boolean ok;
    // The progress in percent, -1 if it is the final reply.
    public int progress = -1;
    @NonNull
    public String error = "";
  }

  private Esp32Protocol() {}

  public static int crc16(@NonNull String text)
  {
    int crc = 0xFFFF;
    for (byte b : text.getBytes(StandardCharsets.US_ASCII))
    {
      crc ^= (b & 0xFF) << 8;
      for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000) != 0 ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
    return crc;
  }

  @NonNull
  public static String command(int id, @NonNull String command)
  {
    final String body = "NGC," + id + "," + command;
    return "$" + body + "*" + String.format(Locale.US, "%04X", crc16(body)) + "\r\n";
  }

  /**
   * @return the fields of a line with a valid checksum, the first one is the line type; null otherwise.
   */
  @Nullable
  public static String[] parse(@NonNull String line)
  {
    final String trimmed = line.trim();
    final int star = trimmed.lastIndexOf('*');
    if (!trimmed.startsWith("$") || star < 0 || trimmed.length() != star + 5)
      return null;
    final String body = trimmed.substring(1, star);
    try
    {
      if (Integer.parseInt(trimmed.substring(star + 1), 16) != crc16(body))
        return null;
    }
    catch (NumberFormatException e)
    {
      return null;
    }
    return body.split(",", -1);
  }

  /**
   * @return the data of an NGD line, null if it is another line or is malformed.
   */
  @Nullable
  public static Data parseData(@NonNull String[] fields)
  {
    if (fields.length < 10 || !"NGD".equals(fields[0]))
      return null;
    try
    {
      if (Integer.parseInt(fields[1]) != VERSION)
        return null;
      final Data data = new Data();
      data.seq = Long.parseLong(fields[2]);
      data.timeMs = Long.parseLong(fields[3]);
      data.yawMdeg = Long.parseLong(fields[4]);
      data.rateMdps = Long.parseLong(fields[5]);
      data.distMm = Long.parseLong(fields[6]);
      data.speedKmh = Integer.parseInt(fields[7]);
      data.speedAgeMs = Integer.parseInt(fields[8]);
      data.flags = Integer.parseInt(fields[9], 16);
      return data;
    }
    catch (NumberFormatException e)
    {
      return null;
    }
  }

  /**
   * @return the reply of an NGA line, null if it is another line or is malformed.
   */
  @Nullable
  public static Reply parseReply(@NonNull String[] fields)
  {
    if (fields.length < 3 || !"NGA".equals(fields[0]))
      return null;
    try
    {
      final Reply reply = new Reply();
      reply.id = Integer.parseInt(fields[1]);
      switch (fields[2])
      {
      case "OK" -> reply.ok = true;
      case "PROGRESS" -> reply.progress = fields.length > 3 ? Integer.parseInt(fields[3]) : 0;
      case "ERR" -> reply.error = fields.length > 3 ? fields[3] : "";
      default -> { return null; }
      }
      return reply;
    }
    catch (NumberFormatException e)
    {
      return null;
    }
  }

  /**
   * @return the event of an NGE line, null if it is another line or is malformed.
   */
  @Nullable
  public static Event parseEvent(@NonNull String[] fields)
  {
    if (fields.length < 7 || !"NGE".equals(fields[0]))
      return null;
    try
    {
      final Event event = new Event();
      event.number = Long.parseLong(fields[2]);
      event.timeMs = Long.parseLong(fields[3]);
      event.level = fields[4];
      event.code = fields[5];
      event.text = fields[6];
      return event;
    }
    catch (NumberFormatException e)
    {
      return null;
    }
  }

  /**
   * @return the number of the last event of the box from an NGS line, -1 if it is another line or the box doesn't
   * tell it.
   */
  public static long parseLastEventNumber(@NonNull String[] fields)
  {
    if (fields.length < 16 || !"NGS".equals(fields[0]))
      return -1;
    try
    {
      return Long.parseLong(fields[15]);
    }
    catch (NumberFormatException e)
    {
      return -1;
    }
  }

  /**
   * @return the state of the OBD adapter from an NGS line: NO_ADAPTER, NO_CAR, OK etc., null if it is another line.
   */
  @Nullable
  public static String parseObdState(@NonNull String[] fields)
  {
    if (fields.length < 6 || !"NGS".equals(fields[0]))
      return null;
    return fields[5];
  }

  /**
   * @return the name of the gyroscope from an NGS line, null if it is another line.
   */
  @Nullable
  public static String parseImuName(@NonNull String[] fields)
  {
    if (fields.length < 4 || !"NGS".equals(fields[0]))
      return null;
    return fields[3];
  }

  /**
   * @return what the box tells about the car in its status, null for another line.
   */
  @Nullable
  public static MotionSource.CarInfo parseCarInfo(@NonNull String[] fields)
  {
    // Fields 17-20 of the status: engine, rpm, the voltage measured by the box and the one from ELM327. Older
    // firmwares send less fields.
    if (fields.length < 17 || !"NGS".equals(fields[0]))
      return null;
    final String engine = fields[16];
    final Boolean running = "RUN".equals(engine) ? Boolean.TRUE : "OFF".equals(engine) ? Boolean.FALSE : null;
    final int boxMillivolts = parseOptionalInt(fields, 18);
    final int elmMillivolts = parseOptionalInt(fields, 19);
    final boolean mismatch = boxMillivolts >= 0 && elmMillivolts >= 0
                          && Math.abs(boxMillivolts - elmMillivolts) > MAX_VOLTAGE_DIFF_MV;
    return new MotionSource.CarInfo(running, parseOptionalInt(fields, 17), boxMillivolts, elmMillivolts, mismatch);
  }

  private static int parseOptionalInt(@NonNull String[] fields, int index)
  {
    if (index >= fields.length || fields[index].isEmpty())
      return -1;
    try
    {
      return Integer.parseInt(fields[index]);
    }
    catch (NumberFormatException e)
    {
      return -1;
    }
  }
}
