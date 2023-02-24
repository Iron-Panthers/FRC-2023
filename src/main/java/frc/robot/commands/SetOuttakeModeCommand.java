// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem.Modes;

public class SetOuttakeModeCommand extends CommandBase {
  private final OuttakeSubsystem outtakeSubsystem;
  private final Modes mode;

  /** Creates a new OuttakeCommand. */
  public SetOuttakeModeCommand(OuttakeSubsystem outtakeSubsystem, Modes mode) {
    this.outtakeSubsystem = outtakeSubsystem;
    this.mode = mode;
    addRequirements(outtakeSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  @Override
  public void initialize() {
    outtakeSubsystem.setMode(mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If we're in a stable state (Hold or Off), OR we have reached a specified end mode, we can end
    return outtakeSubsystem.inStableState();
  }
}
