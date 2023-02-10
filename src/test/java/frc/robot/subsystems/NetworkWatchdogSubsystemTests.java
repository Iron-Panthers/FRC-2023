package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import frc.RobotTest;

public class NetworkWatchdogSubsystemTests {

  @RobotTest
  public void subsystemConstructs() {
    assertDoesNotThrow(NetworkWatchdogSubsystem::new);
  }
}
