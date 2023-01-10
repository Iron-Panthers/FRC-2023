// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import oshi.SystemInfo;
import oshi.software.os.OperatingSystem;

public class DataLogger extends SubsystemBase {

  /** interval in seconds between logging */
  public static final double LOG_INTERVAL = 1;

  private static final SystemInfo systemInfo = new SystemInfo();
  private static final OperatingSystem operatingSystem = systemInfo.getOperatingSystem();

  private double lastLogTime = Timer.getFPGATimestamp();
  /** Creates a new DataLogger. */
  public DataLogger() {
    // no need to construct anything
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();
    double difference = currentTime - lastLogTime;
    if (difference > LOG_INTERVAL) {
      lastLogTime = Timer.getFPGATimestamp();
      createLog();
    }
  }

  private void log(String message) {
    System.out.println("[DataLogger] " + message);
  }

  private void log(String key, Object value) {
    log(key + ": " + value);
  }

  private void log(String key, long value) {
    log(key, String.format("%d", value));
  }

  public void createLog() {
    log("SystemUptime", operatingSystem.getSystemUptime());
  }
}
