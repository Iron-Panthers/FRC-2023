// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class FollowTrajectoryCommand extends CommandBase {
  private final DrivebaseSubsystem drivebaseSubsystem;
  private final Trajectory trajectory;
  /**
   * True if the swerve drive subsystem should localize to the trajectory's starting point in the
   * initialization block. Calls the underlying SwerveDriveOdometry.resetOdometry(pose, angle).
   */
  private final boolean localizeToStartPose;

  /**
   * Creates a new FollowTrajectoryCommand. If you would like to localize to the start pose of the
   * trajectory, instead use the constructor with a boolean parameter.
   *
   * @param trajectory The desired trajectory to track.
   * @param drivebaseSubsystem The instance of the Drivebase subsystem (should come from
   *     RobotContainer)
   */
  public FollowTrajectoryCommand(Trajectory trajectory, DrivebaseSubsystem drivebaseSubsystem) {
    this.trajectory = trajectory;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.localizeToStartPose = false;
    addRequirements(drivebaseSubsystem);
  }

  /**
   * Creates a new FollowTrajectoryCommand. Adds a parameter to optionally localize to the start
   * point of this trajectory.
   *
   * @param trajectory The desired trajectory to track.
   * @param localizeToStartPose If true, the drivebase will reset odometry to trajectory_state[0]
   * @param drivebaseSubsystem The instance of the Drivebase subsystem (should come from
   *     RobotContainer)
   */
  public FollowTrajectoryCommand(
      Trajectory trajectory, boolean localizeToStartPose, DrivebaseSubsystem drivebaseSubsystem) {
    this.trajectory = trajectory;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.localizeToStartPose = localizeToStartPose;
    addRequirements(drivebaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebaseSubsystem.getFollower().follow(trajectory);

    if (localizeToStartPose) {
      // sample the trajectory at 0 seconds (its beginning)
      State firstState = trajectory.sample(0);
      Pose2d pose = firstState.poseMeters;
      if (firstState instanceof PathPlannerState) {
        Rotation2d holonomicRotation = ((PathPlannerState) firstState).holonomicRotation;
        pose = new Pose2d(pose.getTranslation(), holonomicRotation);
      }
      // If it's not an instanceof Pathplanner State, we still need to zero to current position...
      drivebaseSubsystem.resetOdometryToPose(pose);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.getFollower().cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivebaseSubsystem.getFollower().getCurrentTrajectory().isEmpty();
  }
}
