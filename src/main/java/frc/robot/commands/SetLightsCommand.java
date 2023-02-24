// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.RGBSubsystem.RGBColor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLightsCommand extends InstantCommand {
  private final RGBSubsystem rgbSubsystem;
  private final RGBColor color;

  public SetLightsCommand(RGBSubsystem rgbSubsystem, RGBColor color) {
    this.rgbSubsystem = rgbSubsystem;
    this.color = color;
    addRequirements(rgbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rgbSubsystem.showPulseColor(color);
  }
}