// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.Util;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This command takes a drive stick, and the vector of the rotation stick to drive the robot. The
 * rotation stick is made into a vector - if its magnitude is sufficient, its angle is snapped onto
 * the 8 cardinal directions, and the robot is snapped to that angle.
 */
public class RotateVectorDriveCommand extends CommandBase {
  private final DrivebaseSubsystem drivebaseSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationXSupplier;
  private final DoubleSupplier rotationYSupplier;
  private final BooleanSupplier isRobotRelativeSupplier;

  /**
   * The initial angle of the robot at the time of command call. Target angle is added in relation
   * to the initial angle to allow robot-relative drive
   */
  private double initialAngle;

  private double angle = 0;

  private static final double[] angles = {0, 45, 90, 135, 180, 225, 270, 315};

  /** Creates a new DefaultDriveCommand. */
  public RotateVectorDriveCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationXSupplier,
      DoubleSupplier rotationYSupplier,
      BooleanSupplier isRobotRelativeSupplier) {

    this.drivebaseSubsystem = drivebaseSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationXSupplier = rotationXSupplier;
    this.rotationYSupplier = rotationYSupplier;
    this.isRobotRelativeSupplier = isRobotRelativeSupplier;

    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void initialize() {
    angle = drivebaseSubsystem.getGyroscopeRotation().getDegrees();
    initialAngle = angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = translationXSupplier.getAsDouble();
    double y = translationYSupplier.getAsDouble();
    double rotX = rotationXSupplier.getAsDouble();
    double rotY = rotationYSupplier.getAsDouble();
    boolean isRobotRelative = isRobotRelativeSupplier.getAsBoolean();

    double targetAngle = Util.angleSnap(Util.vectorToAngle(-rotX, -rotY), angles);

    // if stick magnitude is greater then rotate angle mag
    if (Util.vectorMagnitude(rotX, rotY) > Drive.ROTATE_VECTOR_MAGNITUDE) {
      angle = isRobotRelative ? Util.normalizeDegrees(targetAngle + initialAngle) : targetAngle;
    }

    drivebaseSubsystem.driveAngle(new Pair<>(x, y), angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivebaseSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // are we at the angle we want
    return Util.epsilonZero(
            Util.relativeAngularDifference(drivebaseSubsystem.getGyroscopeRotation(), angle),
            Drive.ANGULAR_ERROR)
        // is our rotational velocity low
        && Util.epsilonEquals(drivebaseSubsystem.getRotVelocity(), 0, 10)
        // are we not intentionally running pid to hold an angle
        && Util.vectorMagnitude(rotationXSupplier.getAsDouble(), rotationYSupplier.getAsDouble())
            <= Drive.ROTATE_VECTOR_MAGNITUDE;
  }
}
