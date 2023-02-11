package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;
import static org.junit.jupiter.api.condition.OS.LINUX;

import frc.RobotTest;
import frc.robot.Constants.NetworkWatchdog;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.condition.EnabledOnOs;

public class NetworkWatchdogSubsystemTests {
  // a sorta gross way to test if the network is up, but cross platform and doesn't require root or
  // shell usage--preferable to duplicating the code we are testing
  private static boolean isReachable(String addr, int openPort, int timeOutMillis) {
    try (Socket soc = new Socket()) {
      soc.connect(new InetSocketAddress(addr, openPort), timeOutMillis);
      return true;
    } catch (IOException ex) {
      return false;
    }
  }

  // resolution failure is cached so that the test suite doesn't take forever offline
  private static boolean resolved = true;

  private static boolean isOnline() {
    if (!resolved) return false;
    resolved = isReachable("8.8.8.8", 443, 500);
    return resolved;
  }

  @RobotTest
  public void subsystemConstructs() {
    assertDoesNotThrow(NetworkWatchdogSubsystem::new);
  }

  @EnabledOnOs(LINUX)
  @RobotTest
  public void subsystemCanPing() {
    // use assumption to skip test if we don't have network connectivity
    assumeTrue(NetworkWatchdogSubsystemTests::isOnline, "No network connectivity");
    assertTrue(NetworkWatchdogSubsystem.canPing("8.8.8.8"));
  }

  @EnabledOnOs(LINUX)
  @RobotTest
  @Timeout(NetworkWatchdog.PING_TIMEOUT_SECONDS + 1)
  public void subsystemFailsToPing() {
    assumeTrue(NetworkWatchdogSubsystemTests::isOnline, "No network connectivity");
    assertFalse(
        () -> isReachable("192.0.2.0", 443, 500),
        "TEST-NET-1 should not be reachable, it is reserved for documentation and testing");

    assertFalse(NetworkWatchdogSubsystem.canPing("192.0.2.0"));
  }
}
