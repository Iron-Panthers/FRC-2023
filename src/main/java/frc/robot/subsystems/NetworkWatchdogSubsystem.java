// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that monitors the full system network connection, reports status to the shuffleboard,
 * and reboots the network switch attached to the switching port on the pdh to trigger replug events
 * and fix the network.
 */
public class NetworkWatchdogSubsystem extends SubsystemBase {
  /** Creates a new NetworkWatchdogSubsystem. */
  public NetworkWatchdogSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
