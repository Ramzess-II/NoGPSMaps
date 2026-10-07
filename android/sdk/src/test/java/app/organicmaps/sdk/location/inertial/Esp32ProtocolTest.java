package app.organicmaps.sdk.location.inertial;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class Esp32ProtocolTest
{
  @Test
  public void crc()
  {
    // The check value of CRC-16/CCITT-FALSE.
    assertEquals(0x29B1, Esp32Protocol.crc16("123456789"));
  }

  @Test
  public void command()
  {
    assertEquals("$NGC,17,HELLO*D289\r\n", Esp32Protocol.command(17, "HELLO"));
    assertEquals("$NGC,18,CAL_UP*5D27\r\n", Esp32Protocol.command(18, "CAL_UP"));
  }

  @Test
  public void parsesData()
  {
    final String[] fields = Esp32Protocol.parse("$NGD,1,15230,845213,-1234567,-15300,5021345,43,80,1F*1BDE\r\n");
    assertNotNull(fields);
    final Esp32Protocol.Data data = Esp32Protocol.parseData(fields);
    assertNotNull(data);
    assertEquals(15230, data.seq);
    assertEquals(845213, data.timeMs);
    assertEquals(-1234567, data.yawMdeg);
    assertEquals(-15300, data.rateMdps);
    assertEquals(5021345, data.distMm);
    assertEquals(43, data.speedKmh);
    assertEquals(80, data.speedAgeMs);
    assertEquals(0x1F, data.flags);
  }

  @Test
  public void rejectsBadChecksum()
  {
    assertNull(Esp32Protocol.parse("$NGD,1,15230,845213,-1234567,-15300,5021345,43,80,1F*1BDF"));
    assertNull(Esp32Protocol.parse("NGD,1,15230*1BDE"));
    assertNull(Esp32Protocol.parse("$NGD,1,15230"));
    assertNull(Esp32Protocol.parse("$NGD,1*ZZZZ"));
  }

  @Test
  public void rejectsOtherVersion()
  {
    assertNull(Esp32Protocol.parseData(new String[] {"NGD", "2", "1", "1", "1", "1", "1", "1", "1", "1F"}));
  }

  @Test
  public void parsesReplies()
  {
    final String[] ok = Esp32Protocol.parse("$NGA,18,OK*461D");
    assertNotNull(ok);
    final Esp32Protocol.Reply okReply = Esp32Protocol.parseReply(ok);
    assertNotNull(okReply);
    assertEquals(18, okReply.id);
    assertTrue(okReply.ok);

    final String[] progress = Esp32Protocol.parse("$NGA,18,PROGRESS,40*4D95");
    assertNotNull(progress);
    final Esp32Protocol.Reply progressReply = Esp32Protocol.parseReply(progress);
    assertNotNull(progressReply);
    assertEquals(40, progressReply.progress);
    assertFalse(progressReply.ok);

    final String[] error = Esp32Protocol.parse("$NGA,18,ERR,MOVING*416A");
    assertNotNull(error);
    final Esp32Protocol.Reply errorReply = Esp32Protocol.parseReply(error);
    assertNotNull(errorReply);
    assertEquals("MOVING", errorReply.error);
  }

  @Test
  public void parsesStatus()
  {
    final String[] fields = Esp32Protocol.parse("$NGS,1,1.0.0,ICM42688,400,OK,ISO 15765-4 CAN 11/500,12,412,0*8BEB");
    assertNotNull(fields);
    assertEquals("ICM42688", Esp32Protocol.parseImuName(fields));
    assertEquals("OK", Esp32Protocol.parseObdState(fields));
  }

  @Test
  public void parsesStatusWithoutElm327()
  {
    // The fields added in the second spec follow the first ones.
    final String[] fields =
        Esp32Protocol.parse("$NGS,1,2.0.0,ICM42688,400,NO_ADAPTER,,12,412,0,OK,0x47,NO_ADAPTER,0,-15,3*"
                            + String.format("%04X", Esp32Protocol.crc16(
                                  "NGS,1,2.0.0,ICM42688,400,NO_ADAPTER,,12,412,0,OK,0x47,NO_ADAPTER,0,-15,3")));
    assertNotNull(fields);
    assertEquals("NO_ADAPTER", Esp32Protocol.parseObdState(fields));
    assertNull(Esp32Protocol.parseObdState(Esp32Protocol.parse("$NGA,18,OK*461D")));
  }

  @Test
  public void parsesCarInfo()
  {
    // Lines from the firmware with the engine, its speed and the voltage in the fields 17-19.
    MotionSource.CarInfo car = Esp32Protocol.parseCarInfo(Esp32Protocol.parse(
        "$NGS,1,0.2.0,ICM20602,500,OK,ISO 15765-4 (CAN 11/500),12,331,0,OK,0x12,NONE,38400,-386,9,RUN,812,14100*EA0D"));
    assertNotNull(car);
    assertEquals(Boolean.TRUE, car.engineRunning);
    assertEquals(812, car.rpm);
    assertEquals(14100, car.boxMillivolts);
    assertEquals(-1, car.elmMillivolts);
    assertFalse(car.voltageMismatch);

    car = Esp32Protocol.parseCarInfo(
        Esp32Protocol.parse("$NGS,1,0.2.0,ICM20602,500,SLEEP,,12,331,0,OK,0x12,NONE,38400,-386,14,OFF,,12480*3920"));
    assertNotNull(car);
    assertEquals(Boolean.FALSE, car.engineRunning);
    assertEquals(-1, car.rpm);
    assertEquals(12480, car.boxMillivolts);

    // Nothing is known without ELM327: the last fields are empty.
    car = Esp32Protocol.parseCarInfo(
        Esp32Protocol.parse("$NGS,1,0.2.0,ICM20602,500,NO_ADAPTER,,,331,0,OK,0x12,NO_ADAPTER,0,-386,4,,,*D654"));
    assertNotNull(car);
    assertNull(car.engineRunning);
    assertEquals(-1, car.boxMillivolts);

    // Lines of the firmware with 21 fields: the voltage from ELM327 and from the control unit of the car.
    car = Esp32Protocol.parseCarInfo(Esp32Protocol.parse(
        "$NGS,1,0.2.0,ICM20602,500,OK,ISO 15765-4 (CAN 11/500),12,331,0,OK,0x12,NONE,38400,-386,9,RUN,812,14100,13800,14080*AF2F"));
    assertNotNull(car);
    assertEquals(14100, car.boxMillivolts);
    assertEquals(13800, car.elmMillivolts);
    assertEquals(14080, car.ecuMillivolts);
    assertEquals(14080, car.getReferenceMillivolts());
    assertFalse(car.voltageMismatch);

    // The car drives: only the voltage measured by the box comes.
    car = Esp32Protocol.parseCarInfo(Esp32Protocol.parse(
        "$NGS,1,0.2.0,ICM20602,500,OK,ISO 15765-4 (CAN 11/500),3,335,0,OK,0x12,NONE,38400,-380,9,RUN,,14230,,*3483"));
    assertNotNull(car);
    assertEquals(-1, car.rpm);
    assertEquals(14230, car.boxMillivolts);
    assertEquals(-1, car.elmMillivolts);
    assertEquals(-1, car.ecuMillivolts);

    // No divider in the box, the car doesn't tell its voltage.
    car = Esp32Protocol.parseCarInfo(Esp32Protocol.parse(
        "$NGS,1,0.2.0,ICM20602,500,OK,ISO 15765-4 (CAN 11/500),12,331,0,OK,0x12,NONE,38400,-386,9,RUN,790,,13800,*9680"));
    assertNotNull(car);
    assertEquals(-1, car.boxMillivolts);
    assertEquals(13800, car.getReferenceMillivolts());
    assertFalse(car.voltageMismatch);

    // The divider of the box is wrong if its voltage differs from the car by more than 10%.
    final String line = "NGS,1,0.2.0,ICM20602,500,OK,,12,331,0,OK,0x12,NONE,38400,-386,9,RUN,812,17200,12400,";
    car = Esp32Protocol.parseCarInfo(
        Esp32Protocol.parse("$" + line + "*" + String.format("%04X", Esp32Protocol.crc16(line))));
    assertNotNull(car);
    assertTrue(car.voltageMismatch);

    // A status of an older firmware and other lines tell nothing about the car.
    assertNull(Esp32Protocol.parseCarInfo(Esp32Protocol.parse("$NGA,18,OK*461D")));
  }

  @Test
  public void parsesLinesOfFirmware()
  {
    // Lines from the firmware 0.2.0: ELM327 is off, the box stands still.
    final Esp32Protocol.Data data = Esp32Protocol.parseData(Esp32Protocol.parse("$NGD,1,1234,845213,-15300,0,0,-1,-1,227*AEEE"));
    assertNotNull(data);
    assertEquals(-15300, data.yawMdeg);
    assertEquals(-1, data.speedKmh);
    assertEquals(Esp32Protocol.FLAG_IMU_OK | Esp32Protocol.FLAG_BIAS_OK | Esp32Protocol.FLAG_UP_OK
                     | Esp32Protocol.FLAG_STILL | Esp32Protocol.FLAG_OBD_ABSENT,
                 data.flags);

    final String[] status = Esp32Protocol.parse("$NGS,1,0.2.0,ICM20602,500,DISABLED,,,331,0,OK,0x12,NONE,0,-386,3*9ED2");
    assertNotNull(status);
    assertEquals("ICM20602", Esp32Protocol.parseImuName(status));
    assertEquals("DISABLED", Esp32Protocol.parseObdState(status));
    assertEquals(3, Esp32Protocol.parseLastEventNumber(status));

    final Esp32Protocol.Event event = Esp32Protocol.parseEvent(Esp32Protocol.parse("$NGE,1,7,845100,I,CAL_AUTO,-386*AC09"));
    assertNotNull(event);
    assertEquals(7, event.number);
    assertEquals("I", event.level);
    assertEquals("CAL_AUTO", event.code);
    assertEquals("-386", event.text);
  }

  @Test
  public void parsesAccel()
  {
    // The lines of the firmware agent.
    Esp32Protocol.Data data = Esp32Protocol.parseData(
        Esp32Protocol.parse("$NGD,1,5022,945253,-21882,-2100,812584,43,100,1F,-180,95,2350,6800*47CA"));
    assertNotNull(data);
    assertTrue(data.hasAccel);
    assertEquals(-0.18, data.accelH1, 1e-9);
    assertEquals(0.095, data.accelH2, 1e-9);
    assertEquals(2.35, data.accelUp, 1e-9);
    assertEquals(6.8, data.jolt, 1e-9);
    assertEquals(43, data.speedKmh);
    // The vertical is not calibrated.
    data = Esp32Protocol.parseData(Esp32Protocol.parse("$NGD,1,77,12345,0,0,0,-1,-1,223,,,,*9174"));
    assertNotNull(data);
    assertFalse(data.hasAccel);
    // An older box.
    data = Esp32Protocol.parseData(Esp32Protocol.parse("$NGD,1,1234,845213,-15300,0,0,-1,-1,227*AEEE"));
    assertNotNull(data);
    assertFalse(data.hasAccel);
  }
}
