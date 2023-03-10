package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.UtilTest;

public class PoseInversionUtilityTests {
  @UtilTest
  public void originPositionInverts() {
    var origin = new Pose2d(0, 0, new Rotation2d());
    var inverted = PoseInversionUtility.findRedPose(origin);
    assertEquals(
        new Pose2d(FieldObstructionMap.FIELD_LENGTH, 0, Rotation2d.fromDegrees(180)), inverted);
  }

  @UtilTest
  public void scoringPositionInverts() {
    var pose = new Pose2d(1.75, 4.99, Rotation2d.fromDegrees(180));
    var inverted = PoseInversionUtility.findRedPose(pose);
    assertEquals(
        new Pose2d(
            FieldObstructionMap.FIELD_LENGTH - 1.75, 4.99, Rotation2d.fromDegrees(180 + 180)),
        inverted);
  }

  @UtilTest
  public void angledPositionInverts() {
    var pose = new Pose2d(1.75, 4.99, Rotation2d.fromDegrees(30));
    var inverted = PoseInversionUtility.findRedPose(pose);
    assertEquals(
        new Pose2d(FieldObstructionMap.FIELD_LENGTH - 1.75, 4.99, Rotation2d.fromDegrees(180 - 30)),
        inverted);
  }

  @UtilTest
  public void substationPositionInverts() {
    var pose = new Pose2d(15.443, 7.410, Rotation2d.fromDegrees(0));
    var inverted = PoseInversionUtility.findRedPose(pose);
    assertEquals(
        new Pose2d(
            FieldObstructionMap.FIELD_LENGTH - 15.443, 7.410, Rotation2d.fromDegrees(180 + 0)),
        inverted);
  }
}
