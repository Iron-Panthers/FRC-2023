// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.RGBSubsystem.MessagePriority;
import frc.robot.subsystems.RGBSubsystem.PatternTypes;
import frc.robot.subsystems.RGBSubsystem.RGBColor;
import frc.robot.subsystems.RGBSubsystem.RGBMessage;

public class SetRGBTimedCommand extends CommandBase {
  RGBSubsystem rgbSubsystem;
  RGBColor color;
  PatternTypes pattern;
  MessagePriority priority;
  RGBMessage message;
  /** Creates a new SetRGBTimedCommand. */
  public SetRGBTimedCommand(
      RGBSubsystem rgbSubsystem, RGBColor color, PatternTypes pattern, MessagePriority priority) {
    this.rgbSubsystem = rgbSubsystem;
    this.color = color;
    this.pattern = pattern;
    this.priority = priority;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.message = rgbSubsystem.showMessage(color, pattern, priority);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.message.expire();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
