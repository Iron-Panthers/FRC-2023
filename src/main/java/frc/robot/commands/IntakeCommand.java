// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import java.util.function.Supplier;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private Supplier<Modes> modeSupplier;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, Modes mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(intakeSubsystem, () -> mode);
  }

  public IntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Modes> modeSupplier) {
    this.intakeSubsystem = intakeSubsystem;
    this.modeSupplier = modeSupplier;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setMode(modeSupplier.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setMode(modeSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
