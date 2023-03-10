package frc.util.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseInversionUtility {
  private PoseInversionUtility() {}

  private static final Rotation2d mirrorRotation = Rotation2d.fromDegrees(180);

  /**
   * Determine the correct red pose for a given blue side pose. Flip over the x axis center line,
   * and then fix the heading. Field is only mirrored in the x axis.
   */
  public static Pose2d findRedPose(Pose2d bluePose) {
    return bluePose.getTranslation().getX() < FieldObstructionMap.FIELD_CENTER_X
        ? new Pose2d(
            new Translation2d(
                FieldObstructionMap.FIELD_LENGTH - bluePose.getTranslation().getX(),
                bluePose.getTranslation().getY()),
            bluePose.getRotation().plus(mirrorRotation))
        : new Pose2d(
            new Translation2d(
                FieldObstructionMap.FIELD_LENGTH + bluePose.getTranslation().getX(),
                bluePose.getTranslation().getY()),
            bluePose.getRotation().plus(mirrorRotation));
  }
}
