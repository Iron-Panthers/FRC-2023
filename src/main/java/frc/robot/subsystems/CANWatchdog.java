// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANWatchdog extends SubsystemBase {
  private static final String REQUEST = "http://localhost:1250/?action=getdevices";

  private Thread canWatchdogThread;
  private RGBSubsystem rgbSubsystem;

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

  private static void threadFn() {
    while (!Thread.currentThread().isInterrupted()) {}
  }

  /** Creates a new CANWatchdog. */
  public CANWatchdog() {
    canWatchdogThread = new Thread(CANWatchdog::threadFn, "CAN Watchdog Thread");
    canWatchdogThread.setDaemon(true);
    canWatchdogThread.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
