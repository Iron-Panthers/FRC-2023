package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;
import static org.junit.jupiter.api.condition.OS.LINUX;

import frc.RobotTest;
import frc.robot.Constants.NetworkWatchdog;
import frc.robot.subsystems.NetworkWatchdogSubsystem.IPv4;
import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.Optional;
import org.junit.jupiter.api.Disabled;
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
    resolved = isReachable("google.com", 80, 500);
    return resolved;
  }

  private static boolean hasPingBinary() {
    // check if the file /bin/ping exists
    File f = new File("/bin/ping");
    return f.isFile();
  }

  private static boolean isNotCI() {
    return System.getenv("CI") == null;
  }

  @RobotTest
  @Disabled(
      "spams the console, not a very useful test. need to switch to using loggers instead of printing")
  public void subsystemConstructs() {
    assertDoesNotThrow(
        () -> {
          var subsystem = new NetworkWatchdogSubsystem(Optional.empty());
          subsystem.matchStarting();
        });
  }

  @EnabledOnOs(LINUX)
  @RobotTest
  public void subsystemCanPing() {
    // use assumption to skip test if we don't have network connectivity or ping binary
    assumeTrue(
        NetworkWatchdogSubsystemTests::isNotCI,
        "Skipping test on CI, azure does not provide public IP address");
    assumeTrue(NetworkWatchdogSubsystemTests::hasPingBinary, "no ping binary");
    assumeTrue(NetworkWatchdogSubsystemTests::isOnline, "No network connectivity");
    assertTrue(NetworkWatchdogSubsystem.canPing(new IPv4(8, 8, 8, 8)));
  }

  @EnabledOnOs(LINUX)
  @RobotTest
  @Timeout(NetworkWatchdog.PING_TIMEOUT_SECONDS + 1)
  public void subsystemFailsToPing() {
    assumeTrue(
        NetworkWatchdogSubsystemTests::isNotCI,
        "Skipping test on CI, azure does not provide public IP address");
    assumeTrue(NetworkWatchdogSubsystemTests::hasPingBinary, "no ping binary");
    assumeTrue(NetworkWatchdogSubsystemTests::isOnline, "No network connectivity");
    assertFalse(
        () -> isReachable("192.0.2.0", 443, 500),
        "TEST-NET-1 should not be reachable, it is reserved for documentation and testing");

    assertFalse(NetworkWatchdogSubsystem.canPing(new IPv4(192, 0, 2, 0)));
  }
}
