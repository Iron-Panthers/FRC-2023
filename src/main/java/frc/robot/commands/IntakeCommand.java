// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeModes;

/**
 * This command takes a drive stick and nothing else, and drives the robot. It does not send any
 * rotation. This allows the command scheduler to negotiate the different types of rotation without
 * us worrying about it - if a new rotation is introduced, it takes priority, without rotation
 * inside default command always getting lowest priority.
 */
public class IntakeCommand extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;

  private final IntakeModes mode;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeModes mode) {

    this.intakeSubsystem = intakeSubsystem;
    this.mode = mode;

    addRequirements(intakeSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setMode(this.mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMode(IntakeModes.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.getMode() == IntakeModes.OFF;
  }
}
