package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SimpleSwerveTrajectoryFollowerTests {
  private final double kP = 0.01, kI = 0, kD = 0;
  private final double maxVelocity = 4, maxAcceleration = 3;
  private PIDController xPIDController, yPIDController;
  private ProfiledPIDController angleController;
  private SimpleSwerveTrajectoryFollower trajectoryFollower;

  @BeforeEach
  public void setup() {
    xPIDController = new PIDController(kP, kI, kD);
    yPIDController = new PIDController(kP, kI, kD);
    angleController =
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    trajectoryFollower =
        new SimpleSwerveTrajectoryFollower(xPIDController, yPIDController, angleController);
  }

  @Test
  public void simpleSwerveTrajectoryFollowerDefaultValuesAreOk() {
    assertFalse(
        trajectoryFollower.isFinished(), "simple swerve trajectory follower initialized finished");
    assertNull(
        trajectoryFollower.getLastState(),
        "simple swerve trajectory follower initialized with a non-null last-state");
  }

  // for consideration: break apart this test into better logical components
  @Test
  public void callingUpdateUponSimpleSwerveTrajectoryFollowerIsOk() {
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
    Trajectory t =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(1, 2, Rotation2d.fromDegrees(90)),
                new Pose2d(3, 4, Rotation2d.fromDegrees(180))),
            config);

    Pose2d poseAfterFirstPeriodRan = t.sample(2e-2).poseMeters;

    // Just in case, make sure that our trajectory is well-formed
    assertTrue(
        t.getTotalTimeSeconds() > 2e-2, "testing trajectory was not long enough to run for 0.02s");

    // Start following t
    trajectoryFollower.follow(t);

    // run initial update
    trajectoryFollower.update(new Pose2d(), 0, 0);

    Optional<ChassisSpeeds> controlFeedback =
        trajectoryFollower.update(poseAfterFirstPeriodRan, 2e-2, 2e-2);

    assertNotNull(
        trajectoryFollower.getLastState(),
        "last state was null after an update with a non-null Trajectory supplied");
    assertEquals(
        t.sample(2e-2),
        trajectoryFollower.getLastState(),
        "last state variable did not properly sync with the expected value on the supplied trajectory");

    // we are trackIng "perfectly", but controlFeedback should still be non-empty in this specific
    // case. validate.
    assertNotNull(controlFeedback.get(), "control feedback was null");

    assertFalse(
        trajectoryFollower.isFinished(),
        "simple swerve trajectory followed unexpectedly finished early");

    List<Trajectory.State> states = t.getStates();
    Pose2d poseAtEnd = states.get(states.size() - 1).poseMeters;

    trajectoryFollower.update(poseAtEnd, t.getTotalTimeSeconds(), 2e-2);
    assertFalse(trajectoryFollower.isFinished(), "update with time=total_time finished trajectory");

    // Send an update pretending we have just exceeded our trajectory
    controlFeedback = trajectoryFollower.update(poseAtEnd, t.getTotalTimeSeconds() + 2e-2, 2e-2);
    assertTrue(
        trajectoryFollower.isFinished(), "update with time>total_time did not finish trajectory");
    assertEquals(
        new ChassisSpeeds().toString(),
        controlFeedback.get().toString(),
        "update with time>total_time returned non-default feedback speeds");

    trajectoryFollower.cancel();
    assertEquals(
        Optional.empty(),
        trajectoryFollower.getCurrentTrajectory(),
        "cancelling did not clear currentTrajectory");
  }
}
