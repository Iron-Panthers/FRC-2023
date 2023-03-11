package frc.util.pathing;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

public class PoseInversionUtility {
  private PoseInversionUtility() {}

  private static final Rotation2d mirrorRotation = Rotation2d.fromDegrees(180);

  /**
   * Determine the correct red pose for a given blue side pose. Flip over the x axis center line,
   * and then fix the heading. Field is only mirrored in the x axis.
   */
  public static Pose2d findRedPose(Pose2d bluePose) {
    return new Pose2d(
        new Translation2d(
            FieldObstructionMap.FIELD_LENGTH - bluePose.getTranslation().getX(),
            bluePose.getTranslation().getY()),
        mirrorRotation.minus(bluePose.getRotation()));
  }

  /**
   * public static PathPlannerState transformStateForAlliance( PathPlannerState state,
   * DriverStation.Alliance alliance) { if (alliance == DriverStation.Alliance.Red) { // Create a
   * new state so that we don't overwrite the original PathPlannerState transformedState = new
   * PathPlannerState();
   *
   * <p>Translation2d transformedTranslation = new Translation2d(state.poseMeters.getX(),
   * FIELD_WIDTH_METERS - state.poseMeters.getY()); Rotation2d transformedHeading =
   * state.poseMeters.getRotation().times(-1); Rotation2d transformedHolonomicRotation =
   * state.holonomicRotation.times(-1);
   *
   * <p>transformedState.timeSeconds = state.timeSeconds; transformedState.velocityMetersPerSecond =
   * state.velocityMetersPerSecond; transformedState.accelerationMetersPerSecondSq =
   * state.accelerationMetersPerSecondSq; transformedState.poseMeters = new
   * Pose2d(transformedTranslation, transformedHeading); transformedState.angularVelocityRadPerSec =
   * -state.angularVelocityRadPerSec; transformedState.holonomicRotation =
   * transformedHolonomicRotation; transformedState.holonomicAngularVelocityRadPerSec =
   * -state.holonomicAngularVelocityRadPerSec; transformedState.curveRadius = -state.curveRadius;
   * transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter; transformedState.deltaPos
   * = state.deltaPos;
   *
   * <p>return transformedState; } else { return state; } }
   *
   * <p>public static PathPlannerTrajectory transformTrajectoryForAlliance( PathPlannerTrajectory
   * trajectory, DriverStation.Alliance alliance) { if (alliance == DriverStation.Alliance.Red) {
   * List<State> transformedStates = new ArrayList<>();
   *
   * <p>for (State s : trajectory.getStates()) { PathPlannerState state = (PathPlannerState) s;
   *
   * <p>transformedStates.add(transformStateForAlliance(state, alliance)); }
   *
   * <p>return new PathPlannerTrajectory( transformedStates, trajectory.markers,
   * trajectory.startStopEvent, trajectory.endStopEvent, trajectory.fromGUI); } else { return
   * trajectory; } }
   */
  public static PathPlannerState transformStateForRed(PathPlannerState state) {
    // Create a new state so that we don't overwrite the original PathPlannerState
    PathPlannerState transformedState = new PathPlannerState();

    Translation2d transformedTranslation =
        new Translation2d(
            FieldObstructionMap.FIELD_LENGTH - state.poseMeters.getX(), state.poseMeters.getY());
    Rotation2d transformedHeading = mirrorRotation.minus(state.poseMeters.getRotation());
    Rotation2d transformedHolonomicRotation = mirrorRotation.minus(state.holonomicRotation);

    transformedState.timeSeconds = state.timeSeconds;
    transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
    transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
    transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
    transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
    transformedState.holonomicRotation = transformedHolonomicRotation;
    transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
    try {
      // use reflection to read and write private field
      // transformedState.curveRadius = -state.curveRadius;

      Field curveRadiusField = PathPlannerState.class.getDeclaredField("curveRadius");
      curveRadiusField.setAccessible(true);
      curveRadiusField.setDouble(transformedState, -curveRadiusField.getDouble(state));

      transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
      // use reflection to read and write private field
      // transformedState.deltaPos = state.deltaPos;

      Field deltaPosField = PathPlannerState.class.getDeclaredField("deltaPos");
      deltaPosField.setAccessible(true);
      deltaPosField.setDouble(transformedState, deltaPosField.getDouble(state));

    } catch (NoSuchFieldException | IllegalAccessException e) {
      // FIXME: handle exception?
      e.printStackTrace();
    }

    return transformedState;
  }

  public static PathPlannerTrajectory transformTrajectoryForRed(PathPlannerTrajectory trajectory) {
    List<State> transformedStates = new ArrayList<>();

    for (State s : trajectory.getStates()) {
      PathPlannerState state = (PathPlannerState) s;

      transformedStates.add(transformStateForRed(state));
    }

    try {
      Field markersField = PathPlannerTrajectory.class.getDeclaredField("markers");
      markersField.setAccessible(true);
      Field startStopEventField = PathPlannerTrajectory.class.getDeclaredField("startStopEvent");
      startStopEventField.setAccessible(true);
      Field endStopEventField = PathPlannerTrajectory.class.getDeclaredField("endStopEvent");
      endStopEventField.setAccessible(true);

      return new PathPlannerTrajectory(
          transformedStates,
          (List<EventMarker>) markersField.get(trajectory),
          (StopEvent) startStopEventField.get(trajectory),
          (StopEvent) endStopEventField.get(trajectory),
          trajectory.fromGUI);
    } catch (NoSuchFieldException | IllegalAccessException e) {
      e.printStackTrace();
      return new PathPlannerTrajectory();
    }
  }
}
