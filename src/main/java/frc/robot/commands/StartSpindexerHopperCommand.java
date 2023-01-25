// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.SpindexerHopperSubsystem;
import frc.robot.subsystems.SpindexerHopperSubsystem.Modes;

public class StartSpindexerHopperCommand extends CommandBase {
  SpindexerHopperSubsystem spindexerHopperSubsystem;
  Modes mode;

  /** Creates a new DefenseModeCommand. */
  public StartSpindexerHopperCommand(SpindexerHopperSubsystem spindexerHopperSubsystem) {
    this.spindexerHopperSubsystem = spindexerHopperSubsystem;

    this.mode = Modes.IDLE;

    addRequirements(spindexerHopperSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexerHopperSubsystem.setMode(mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return spindexerHopperSubsystem.getMode() == Modes.OFF;
  }
}
