// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.RGBSubsystem.RGBColor;
import frc.robot.subsystems.RGBSubsystem.RGBMessage;
import java.util.Optional;

public class ForceLightsColorCommand extends CommandBase {

  private final RGBSubsystem rgbSubsystem;
  private final RGBColor color;
  private Optional<RGBMessage> message = Optional.empty();

  /** Creates a new ForceLightsColorCommand. */
  public ForceLightsColorCommand(RGBSubsystem rgbSubsystem, RGBColor color) {
    this.rgbSubsystem = rgbSubsystem;
    this.color = color;
    addRequirements(rgbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    message =
        Optional.of(
            rgbSubsystem.showMessage(
                color,
                RGBSubsystem.PatternTypes.PULSE,
                RGBSubsystem.MessagePriority.C_DRIVER_CONTROLLED_COLOR));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    message.ifPresent(RGBMessage::expire);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
