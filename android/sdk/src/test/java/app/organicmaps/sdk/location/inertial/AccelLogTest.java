package app.organicmaps.sdk.location.inertial;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import org.junit.Test;

public class AccelLogTest
{
  private static final long STEP_NS = 20_000_000;

  @Test
  public void lineEverySecond()
  {
    final AccelLog log = new AccelLog();
    String line = null;
    int lines = 0;
    // 50 samples a second for 3 s and one more to end the last second.
    for (int i = 0; i <= 150; i++)
    {
      // A jolt of 3 m/s² up in the third part of the second second.
      final float z = i == 75 ? 12.81f : 9.81f;
      final String result = log.onSample(i * STEP_NS, 0.5f, -1, z);
      if (result == null)
        continue;
      lines++;
      if (lines == 2)
        line = result;
    }
    assertEquals(3, lines);
    assertNotNull(line);
    final String[] slices = line.split(" ");
    assertEquals(AccelLog.SLICES, slices.length);
    assertEquals("0.50,-1.00,9.81,0.0", slices[0]);
    // The mean of the part with the jolt is 0.3 higher, the jolt is 2.7 above it.
    assertEquals("0.50,-1.00,10.11,2.7", slices[2]);
  }

  @Test
  public void breakStartsAnew()
  {
    final AccelLog log = new AccelLog();
    for (int i = 0; i < 40; i++)
      assertNull(log.onSample(i * STEP_NS, 0, 0, 9.81f));
    // The sensor was silent for 5 s: the unfinished second is dropped.
    final long later = 5_000_000_000L;
    for (int i = 0; i < 50; i++)
      assertNull(log.onSample(later + i * STEP_NS, 0, 0, 9.81f));
    assertNotNull(log.onSample(later + 50 * STEP_NS, 0, 0, 9.81f));
  }
}
