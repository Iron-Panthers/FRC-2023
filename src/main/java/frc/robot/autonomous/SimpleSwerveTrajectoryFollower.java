// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.util.Util;

/**
 * Implements a simple swerve trajectory follower, with a fixed robot heading.
 *
 * <p>See TrajectoryFollower class for more detailed documentation.
 */
public class SimpleSwerveTrajectoryFollower extends TrajectoryFollower<ChassisSpeeds> {
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController angleController;

  private Trajectory.State lastState = null;
  private boolean finished = false;
  /* Variable to track if calculateDriveSignal has run once yet */
  private boolean firstRun = false;

  public SimpleSwerveTrajectoryFollower(
      PIDController xController, PIDController yController, ProfiledPIDController angleController) {
    this.xController = xController;
    this.yController = yController;
    this.angleController = angleController;
  }

  @Override
  protected ChassisSpeeds calculateDriveSignal(
      Pose2d currentPose, Trajectory trajectory, double time, double dt) {
    if (time > trajectory.getTotalTimeSeconds()) {
      finished = true;
      return new ChassisSpeeds();
    }

    // there is still time left!
    lastState = trajectory.sample(time);
    if (firstRun) {
      angleController.reset(currentPose.getRotation().getRadians());
      firstRun = false;
    }
    Pose2d poseRef = lastState.poseMeters;
    double linearVelocityRefMeters = lastState.velocityMetersPerSecond;
    double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
    double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();

    double currentDegrees = (360 - currentPose.getRotation().getDegrees()) % 360;
    double targetDegrees = (360 - poseRef.getRotation().getDegrees()) % 360;

    // scope current and target angles
    double angularDifferenceDeg = Util.relativeAngularDifference(currentDegrees, targetDegrees);

    // SmartDashboard.putNumber("current pose rotation", currentDegrees);
    // SmartDashboard.putNumber("pose ref rotation", targetDegrees);
    // SmartDashboard.putNumber("sstf/angular_difference (deg)", angularDifferenceDeg);

    // use pid controller using scoped angles to get shortest-distance correction
    double angleFF = angleController.calculate(angularDifferenceDeg, 0);

    // SmartDashboard.putNumber("angleff", angleFF);

    double xControllerEffort = xController.calculate(currentPose.getX(), poseRef.getX());
    double yControllerEffort = yController.calculate(currentPose.getY(), poseRef.getY());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xControllerEffort, yFF + yControllerEffort, angleFF, currentPose.getRotation());
  }

  public Trajectory.State getLastState() {
    return lastState;
  }

  @Override
  protected boolean isFinished() {
    return finished;
  }

  @Override
  protected void reset() {
    xController.reset();
    yController.reset();
    angleController.reset(0);
    finished = false;
    firstRun = false;
  }
}
