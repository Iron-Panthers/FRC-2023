// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeModeCommand extends InstantCommand {
  IntakeSubsystem intakeSubsystem;
  IntakeSubsystem.Modes intakeMode;

  public SetIntakeModeCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.Modes intakeMode) {
    this.intakeMode = intakeMode;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setMode(intakeMode);
  }
}
