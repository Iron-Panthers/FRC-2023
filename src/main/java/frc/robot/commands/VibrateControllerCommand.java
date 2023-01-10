// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VibrateControllerCommand extends CommandBase {
  private XboxController controller;
  private double duration;
  private double startTime;
  private double strength;

  /**
   * makes a command to vibrate a controller
   *
   * @param controller the controller to call the rumble methods on
   * @param duration the time (seconds) to vibrate for
   * @param strength the strength [0, 1] double of the vibration
   */
  public VibrateControllerCommand(XboxController controller, double duration, double strength) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.duration = duration;
    this.strength = strength;
    startTime = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    controller.setRumble(RumbleType.kRightRumble, strength);
    controller.setRumble(RumbleType.kLeftRumble, strength);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kRightRumble, 0);
    controller.setRumble(RumbleType.kLeftRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= startTime + duration;
  }
}
