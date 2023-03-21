// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.Config;
import frc.robot.autonomous.TrajectoryFollower;

/**
 * Implements a simple swerve trajectory follower, with a fixed robot heading.
 *
 * <p>See TrajectoryFollower class for more detailed documentation.
 */
public class AdvancedSwerveTrajectoryFollower extends TrajectoryFollower<ChassisSpeeds> {
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController angleController;

  private Trajectory.State lastState = null;
  private boolean finished = false;
  /* Variable to track if calculateDriveSignal has run once yet */
  private boolean firstRun = false;

  public AdvancedSwerveTrajectoryFollower(
      PIDController xController, PIDController yController, ProfiledPIDController angleController) {
    this.xController = xController;
    // var tab = Shuffleboard.getTab("tuneFollower");
    // tab.add(xController);
    // tab.add(yController);
    // tab.addDouble("xError", xController::getPositionError);
    // tab.addDouble("yError", yController::getPositionError);
    this.yController = yController;
    this.angleController = angleController;
  }

  private ChassisSpeeds finishTrajectory() {
    finished = true;
    return new ChassisSpeeds();
  }

  @Override
  protected ChassisSpeeds calculateDriveSignal(
      Pose2d currentPose, Trajectory trajectory, double time, double dt) {
    if (time > trajectory.getTotalTimeSeconds()) {
      // If the robot is past the end of the trajectory, stop
      return finishTrajectory();
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
    double targetDegrees =
        // only cast if safe
        lastState instanceof PathPlannerState
            ? ((360 - ((PathPlannerState) lastState).holonomicRotation.getDegrees()) % 360)
            : ((360 - poseRef.getRotation().getDegrees()) % 360);

    if (Config.RUN_PATHPLANNER_SERVER)
      PathPlannerServer.sendPathFollowingData(
          new Pose2d(poseRef.getTranslation(), Rotation2d.fromDegrees(targetDegrees)), currentPose);

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
