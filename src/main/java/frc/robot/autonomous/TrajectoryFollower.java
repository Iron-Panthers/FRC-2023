// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.Config;
import java.util.Optional;

/**
 * Follower for time-parametrized trajectories, providing output signals of type T.
 *
 * <p>"Output signals" means information that the relevant mechanism (in this case, the drivebase)
 * can use in order to achieve their desired trajectory states during operation. For example, in the
 * case of the SimpleSwerveTrajectoryFollower, these are of type ChassisSpeeds, which defines the
 * translation and rotation speeds of the robot -- values which can then be used to write outputs to
 * the drivebase and follow the driving path.
 */
public abstract class TrajectoryFollower<T> {
  /**
   * The trajectory that is currently being followed. null if no trajectory is being followed.
   *
   * <p>If the value of currentTrajectory is null, your follower's #update method will return an
   * empty Optional when called (corresponding to an undefined output).
   */
  private Trajectory currentTrajectory = null;

  /**
   * The time that the current trajectory started to be followed. NaN if the trajectory has not been
   * started yet.
   */
  private double startTime = Double.NaN;

  /**
   * Calculates the output required to follow the trajectory.
   *
   * @param currentPose the current pose of the robot
   * @param trajectory the trajectory to follow
   * @param time the amount of time that has elapsed since the current trajectory started to be
   *     followed
   * @param dt the amount of time that has elapsed since the update loop was last ran
   * @return the chassis speeds required to follow the trajectory
   */
  protected abstract T calculateDriveSignal(
      Pose2d currentPose, Trajectory trajectory, double time, double dt);

  /**
   * Gets if the follower is done following the path.
   *
   * @return true if the path is done
   */
  protected abstract boolean isFinished();

  /**
   * Resets the TrajectoryFollower for first run. This likely involves resetting integrators,
   * controllers, or any other relevant tracking variables which should map 1:1 with each Trajectory
   */
  protected abstract void reset();

  /** Sets the current Trajectory to follow to null, stopping operation. */
  public final void cancel() {
    currentTrajectory = null;
  }

  /**
   * Sets the follower to follow a certain trajectory. This will reset the start time so you
   * shouldn't call this while following a trajectory.
   *
   * @param trajectory The desired Trajectory to follow.
   */
  public final void follow(Trajectory trajectory) {
    currentTrajectory = trajectory;
    startTime = Double.NaN;
    if (Config.RUN_PATHPLANNER_SERVER) PathPlannerServer.sendActivePath(trajectory.getStates());
  }

  /** Gets the current trajectory that is being followed, if applicable. */
  public final Optional<Trajectory> getCurrentTrajectory() {
    return Optional.ofNullable(currentTrajectory);
  }

  /**
   * Gets the time at which the current trajectory was started; NaN if not started or no trajectory.
   */
  public final double getStartTime() {
    return startTime;
  }

  /**
   * Gets the desired output for the robot at this time.
   *
   * @param currentPose the current pose of the robot
   * @param time the current time
   * @param dt the time since update was last called
   * @return the output required to follow the current path if applicable
   */
  public final Optional<T> update(Pose2d currentPose, double time, double dt) {
    Trajectory trajectory;
    double timeSinceStart;
    // Return empty if no trajectory is being followed
    if (currentTrajectory == null) {
      return Optional.empty();
    }
    // If the trajectory has not been started, update the start time and reset the follower state
    if (Double.isNaN(startTime)) {
      startTime = time;
      reset();
    } else if (isFinished()) {
      currentTrajectory = null;
      return Optional.empty();
    }
    trajectory = currentTrajectory;
    timeSinceStart = time - startTime;

    T speeds = calculateDriveSignal(currentPose, trajectory, timeSinceStart, dt);
    return Optional.of(speeds);
  }
}
