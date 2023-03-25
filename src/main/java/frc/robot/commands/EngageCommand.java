// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive.AutoBalance;
import frc.robot.subsystems.DrivebaseSubsystem;

/*
private void balancePeriodic() {
    // x direction
    double roll = navx.getRoll();
    double absRoll = Math.abs(roll);
    // y direction
    double pitch = navx.getPitch();
    double absPitch = Math.abs(pitch);

    if (Math.max(absRoll, absPitch) <= AutoBalance.THRESHOLD_ANGLE) {
      defensePeriodic();
      return;
    }

    double control =
        AutoBalance.P_SPEED_METERS_PER_SECOND
            * Math.pow(
                MathUtil.clamp(Math.max(absRoll, absPitch) / AutoBalance.MAX_ANGLE, 0, 1),
                AutoBalance.E_EXPONENTIAL_FACTOR);

    // use bang bang to generate chassis speeds that will balance the robot
    chassisSpeeds =
        absRoll > absPitch
            ? new ChassisSpeeds(Math.copySign(control, roll), 0, 0)
            : new ChassisSpeeds(0, Math.copySign(control, pitch), 0);

    drivePeriodic();
  }
 */

public class EngageCommand extends CommandBase {
  DrivebaseSubsystem drivebaseSubsystem;
  double maxRoll = 0;

  /** Creates a new EngageCommand. */
  public EngageCommand(DrivebaseSubsystem drivebaseSubsystem) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    addRequirements(drivebaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    maxRoll = drivebaseSubsystem.getRollPitch().absRoll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // pitch is x
    var rollPitch = drivebaseSubsystem.getRollPitch();
    maxRoll = Math.max(maxRoll, rollPitch.absRoll());
    if (maxRoll > AutoBalance.DOCK_SPEED_UNTIL_ANGLE_DEGREES
        && rollPitch.absRoll() > AutoBalance.ENGAGE_SPEED_UNTIL_ANGLE_DEGREES) {
      drivebaseSubsystem.drive(
          new ChassisSpeeds(
              Math.copySign(AutoBalance.ENGAGE_SPEED_METERS_PER_SECOND, rollPitch.roll()), 0, 0));
    } else if (maxRoll < AutoBalance.DOCK_SPEED_UNTIL_ANGLE_DEGREES) {
      drivebaseSubsystem.drive(
          new ChassisSpeeds(
              Math.copySign(AutoBalance.DOCK_SPEED_METERS_PER_SECOND, rollPitch.roll()), 0, 0));
    } else {
      drivebaseSubsystem.setDefenseMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.setDefenseMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
