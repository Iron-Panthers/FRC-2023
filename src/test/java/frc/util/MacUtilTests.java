package frc.util;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.UtilTest;
import java.util.Arrays;

public class MacUtilTests {

  private byte[] macOne = {46, -75, -30, 99, 104, 4};
  private String macOneString = "2E:B5:E2:63:68:04";
  private byte[] macTwo = {94, 68, 23, 53, 15, -37};
  private String macTwoString = "5E:44:17:35:0F:DB";

  @UtilTest
  public void getMacAddressesDoesNotThrow() {
    assertDoesNotThrow(() -> MacUtil.getMacAddress());
  }

  @UtilTest
  public void macAddressIsFormattedProperly() {
    assertEquals(macOneString, MacUtil.macToString(macOne));
    assertEquals(macTwoString, MacUtil.macToString(macTwo));
  }

  @UtilTest
  public void macAddressFormatsCorrectlyWithEmptyByteArray() {
    assertEquals("", MacUtil.macToString(new byte[0]));
    assertEquals("00:00:00:00:00:00", MacUtil.macToString(new byte[6]));
  }

  @UtilTest
  public void similarMacAddressesNotEqual() {
    byte[] evilMac = Arrays.copyOf(macTwo, macTwo.length);
    evilMac[2]++;
    assertNotEquals(macTwoString, MacUtil.macToString(evilMac));
  }

  @UtilTest
  public void compBotDefaultsTrue() {
    assertTrue(MacUtil.IS_COMP_BOT);
  }
}
