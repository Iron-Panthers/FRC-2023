// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ForceIntakeModeCommand extends CommandBase {
  IntakeSubsystem intakeSubsystem;
  IntakeSubsystem.Modes intakeMode;

  /** Creates a new ForceIntakeModeCommand. */
  public ForceIntakeModeCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.Modes intakeMode) {
    this.intakeMode = intakeMode;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setModeLocked(true);
    intakeSubsystem.setMode(intakeMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setModeLocked(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
