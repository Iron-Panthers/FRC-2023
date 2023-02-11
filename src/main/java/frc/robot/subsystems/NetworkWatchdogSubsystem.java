// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NetworkWatchdog;
import java.io.IOException;

/**
 * A subsystem that monitors the full system network connection, reports status to the shuffleboard,
 * and reboots the network switch attached to the switching port on the pdh to trigger replug events
 * and fix the network.
 */
public class NetworkWatchdogSubsystem extends SubsystemBase {

  /**
   * Blocking method that tests if /bin/ping can reach the specified host. You should never call
   * this on the main thread.
   *
   * @param host the host to ping
   * @return true if the host is reachable, false otherwise
   */
  public static boolean canPing(final String host) {
    try {
      Process process =
          new ProcessBuilder(
                  "/bin/ping",
                  "-c",
                  "1",
                  "-W",
                  String.valueOf(NetworkWatchdog.PING_TIMEOUT_SECONDS),
                  host)
              .redirectErrorStream(true)
              .start();

      return process.waitFor() == 0;

    } catch (IOException e) {
      throw new IllegalArgumentException(e);
    } catch (InterruptedException e) {
      // restore the interrupted status
      Thread.currentThread().interrupt();
    }
    return false;
  }

  /** Creates a new NetworkWatchdogSubsystem. */
  public NetworkWatchdogSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Stop rebooting the network switch. */
  public void matchStarting() {}
}
