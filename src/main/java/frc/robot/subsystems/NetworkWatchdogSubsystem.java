// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NetworkWatchdog;
import java.io.IOException;
import oshi.SystemInfo;

/**
 * A subsystem that monitors the full system network connection, reports status to the shuffleboard,
 * and reboots the network switch attached to the switching port on the pdh to trigger replug events
 * and fix the network.
 */
public class NetworkWatchdogSubsystem extends SubsystemBase {
  private Thread networkWatchdogThread;
  private PowerDistribution pdh = new PowerDistribution();

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

  /**
   * Sleep that handles interrupts and uses an int. Don't do this please?
   *
   * @param millis
   */
  private static void sleep(final int millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      // restore the interrupted status
      Thread.currentThread().interrupt();
    }
  }

  /** Creates a new NetworkWatchdogSubsystem. */
  public NetworkWatchdogSubsystem() {
    pdh.setSwitchableChannel(true);
    networkWatchdogThread =
        new Thread(
            () -> {
              final int initialUptimeMS =
                  (int) Math.floor(new SystemInfo().getOperatingSystem().getSystemUptime() * 1000d);
              if (initialUptimeMS < NetworkWatchdog.BOOT_SCAN_DELAY_MS) {
                sleep(NetworkWatchdog.BOOT_SCAN_DELAY_MS - initialUptimeMS);
              }

              // to ensure we don't miss an interrupt, only sleep once per branch before coming back
              // to the while conditional
              while (!Thread.interrupted()) {
                if (canPing(NetworkWatchdog.TEST_IP_ADDRESS)) {
                  System.out.println(
                      "[network watchdog] Pinged "
                          + NetworkWatchdog.TEST_IP_ADDRESS
                          + " successfully.");
                  sleep(NetworkWatchdog.HEALTHY_CHECK_INTERVAL_MS);
                } else {
                  System.out.println(
                      "[network watchdog] Failed to ping "
                          + NetworkWatchdog.TEST_IP_ADDRESS
                          + ", rebooting switch.");
                  pdh.setSwitchableChannel(false);
                  sleep(NetworkWatchdog.REBOOT_DURATION_MS);
                  pdh.setSwitchableChannel(true);
                  System.out.println("[network watchdog] Switch rebooted. Waiting.");
                  sleep(NetworkWatchdog.SWITCH_POWERCYCLE_SCAN_DELAY_MS);
                }
              }
            });
    networkWatchdogThread.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Stop rebooting the network switch. */
  public void matchStarting() {
    // stop the thread
    networkWatchdogThread.interrupt();
    System.out.println("[network watchdog] Network watchdog thread stopped.");
    // always reenable switchable channel after killing the thread
    pdh.setSwitchableChannel(true);
  }
}
