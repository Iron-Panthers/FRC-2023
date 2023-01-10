// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.Util;
import java.util.function.DoubleSupplier;

/**
 * This command takes a drive angle and a target angle, and snaps the robot to an angle. This is
 * useful to snap the robot to an angle setpoint with a button, as opposed to using an entire stick.
 */
public class RotateAngleDriveCommand extends CommandBase {
  private final DrivebaseSubsystem drivebaseSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  private final int targetAngle;

  /** Creates a new RotateAngleDriveCommand. */
  public RotateAngleDriveCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      int targetAngle) {

    this.drivebaseSubsystem = drivebaseSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;

    this.targetAngle = targetAngle;

    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = translationXSupplier.getAsDouble();
    double y = translationYSupplier.getAsDouble();

    drivebaseSubsystem.driveAngle(
        new Pair<Double, Double>(x, y), targetAngle // the desired angle, gyro relative
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.epsilonZero(
            Util.relativeAngularDifference(drivebaseSubsystem.getGyroscopeRotation(), targetAngle),
            Drive.ANGULAR_ERROR)
        && Util.epsilonEquals(drivebaseSubsystem.getRotVelocity(), 0, 10);
  }
}
