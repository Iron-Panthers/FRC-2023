// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem.Modes;
import java.util.Optional;

public class OuttakeCommand extends CommandBase {
  private final OuttakeSubsystem outtakeSubsystem;
  private final Modes mode;
  private final Optional<Modes> endingMode;

  /** Creates a new OuttakeCommand. */
  public OuttakeCommand(OuttakeSubsystem outtakeSubsystem, Modes mode, Optional<Modes> endingMode) {
    this.outtakeSubsystem = outtakeSubsystem;
    this.mode = mode;
    this.endingMode = endingMode;
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
  public void end(boolean interrupted) {
    // Should we set the state to either be the hold or open "stable" states?
    // Would allow drivers to have more control...
    if (endingMode.isPresent()) outtakeSubsystem.setMode(endingMode.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean reachEndMode = false;
    if (endingMode.isPresent()) reachEndMode = outtakeSubsystem.getMode() == endingMode.get();

    return outtakeSubsystem.inStableState() || reachEndMode;
  }
}
