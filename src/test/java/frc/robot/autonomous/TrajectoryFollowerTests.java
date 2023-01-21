package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TrajectoryFollowerTests {
  // The naïve mock object
  private ConcreteImpl concreteTrajectoryFollower;

  // Basic concrete implementation of TrajectoryFollower to allow for testing of the concrete
  // methods
  // NOTE: this is probably better implemented via Mockito.
  private class ConcreteImpl extends TrajectoryFollower<Integer> {
    /**
     * @return -1
     */
    @Override
    protected Integer calculateDriveSignal(
        Pose2d currentPose, Trajectory trajectory, double time, double dt) {
      return -1;
    }

    // Backing value for isFinished to represent a finish condition (usually passing a certain point
    // or timestamp or whatnot)
    private boolean _finished = false;

    /**
     * @return value of _finished
     */
    @Override
    protected boolean isFinished() {
      return _finished;
    }

    public void markFinished() {
      _finished = true;
    }

    @Override
    protected void reset() {
      _finished = false;
    }
  }

  @BeforeEach
  public void setup() {
    concreteTrajectoryFollower = new ConcreteImpl();
  }

  @Test
  public void testCurrentTrajectoryUpdates() {
    // should initialize as null
    assertEquals(
        Optional.empty(),
        concreteTrajectoryFollower.getCurrentTrajectory(),
        "currentTrajectory not empty");

    // let's update the value of currentTrajectory
    Trajectory t = new Trajectory();
    concreteTrajectoryFollower.follow(t);

    assertEquals(
        Optional.of(t),
        concreteTrajectoryFollower.getCurrentTrajectory(),
        "currentTrajectory did not update");

    // let's force trajectory to finish then call update -- currentTrajectory should be cleared on
    // next update
    // replicates having reached the finish condition (pose, time, or other)
    concreteTrajectoryFollower.markFinished();

    // we should be considered finished
    assertTrue(
        concreteTrajectoryFollower.isFinished(),
        "markFinished in trajectory follower mock did not work");

    // calling markFinished should not have affected the currentTrajectory value
    assertEquals(
        Optional.of(t),
        concreteTrajectoryFollower.getCurrentTrajectory(),
        "currentTrajectory unexpectedly disappeared");

    // send update
    // This should update the isFinished value
    concreteTrajectoryFollower.update(new Pose2d(), 0, 0);

    assertFalse(
        concreteTrajectoryFollower.isFinished(),
        "first run of follower update didn't change finished value");

    // cancelling should clear our currentTrajectory
    concreteTrajectoryFollower.cancel();
    assertEquals(
        Optional.empty(),
        concreteTrajectoryFollower.getCurrentTrajectory(),
        "cancelling did not clear the current trajectory");
  }

  @Test
  public void testStartTimeBehaviorWithIsFinishedAndUpdates() {
    // value should start as false
    assertFalse(
        concreteTrajectoryFollower.isFinished(), "trajectory follower mock initialized finished");
    // value should start as NaN
    assertTrue(
        Double.isNaN(concreteTrajectoryFollower.getStartTime()),
        "trajectory follower mock initialized with non-NaN start time");

    // replicates having reached the finish condition (pose, time, or other)
    concreteTrajectoryFollower.markFinished();
    assertTrue(
        concreteTrajectoryFollower.isFinished(),
        "trajectory follower mock didn't properly update finished state");
    // value should stay as NaN since we never called update
    assertTrue(
        Double.isNaN(concreteTrajectoryFollower.getStartTime()),
        "start time unexpectedly changed (before update)");

    // send update
    // since no trajectory supplied, this should simply return an empty optional without resetting
    assertEquals(Optional.empty(), concreteTrajectoryFollower.update(new Pose2d(), 0, 0));
    assertTrue(
        Double.isNaN(concreteTrajectoryFollower.getStartTime()),
        "start time unexpectedly changed after updating without trajectory");

    Trajectory t = new Trajectory();

    // We have validated that currentTrajectory is non-null after follow(t) in the above test
    concreteTrajectoryFollower.follow(t);

    // Now that trajectory is non-null, the next update(Pose2d, double, double) should call
    // the contents of reset(): which should, for our implementation, reset isFinished 's backing
    // value to false
    // also, the return should be an optional based on the calculateDriveSignal call - which for all
    // concrete impl means that we expect to see an Option of -1
    assertEquals(
        Optional.of(-1),
        concreteTrajectoryFollower.update(new Pose2d(), 1, 1),
        "trajectory follower mock calculateDriveSignal did not have expected output");

    // start time should have been updated to time (1) in the above call. validate
    assertEquals(
        1, concreteTrajectoryFollower.getStartTime(), "start time did not change after update");

    // reset method should have been called. validate
    assertFalse(
        concreteTrajectoryFollower.isFinished(), "trajectory followed unexpectedly finished early");
  }
}
