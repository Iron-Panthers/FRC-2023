package frc.util.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

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

  public static Trajectory.State findRedState(Trajectory.State blueState) {
    return new Trajectory.State(
        blueState.timeSeconds,
        blueState.velocityMetersPerSecond,
        blueState.accelerationMetersPerSecondSq,
        findRedPose(blueState.poseMeters),
        blueState.curvatureRadPerMeter);
  }
}
