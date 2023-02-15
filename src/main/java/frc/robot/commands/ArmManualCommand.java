// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmManualCommand extends CommandBase {
  ArmSubsystem armSubsystem;
  DoubleSupplier angleSupplier;
  DoubleSupplier extensionSupplier;
  /** Creates a new AngleArmCommand. */
  public ArmManualCommand(
      ArmSubsystem armSubsystem, DoubleSupplier angleSupplier, DoubleSupplier extensionSupplier) {
    this.armSubsystem = armSubsystem;

    this.angleSupplier = angleSupplier;
    this.extensionSupplier = extensionSupplier;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angleOutput = angleSupplier.getAsDouble();
    double extensionOutput = extensionSupplier.getAsDouble();

    armSubsystem.setTargetPosition(
        armSubsystem.getTargetAngleDegrees() + angleOutput,
        armSubsystem.getTargetExtensionInches() + extensionOutput);
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
