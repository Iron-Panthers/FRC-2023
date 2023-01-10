package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.UtilTest;

public class ControllerUtilTests {
  @UtilTest
  public void deadbandScalesCorrectly() {
    assertEquals(1.0, ControllerUtil.deadband(1.0, .3), "deadband should still max out");
    assertEquals(0.0, ControllerUtil.deadband(.1, .2), "value should be zero when inside deadband");
    assertEquals(
        0.2,
        ControllerUtil.deadband(.6, .5),
        1e-5 // epsilon for equality assertion
        ,
        "value should scale such that range from deadband to 1 is mapped to 0 to 1");
    assertEquals(-.2, ControllerUtil.deadband(-.6, .5), 1e-5, "works for negative numbers");
  }
}
