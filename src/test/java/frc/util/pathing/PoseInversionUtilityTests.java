package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import au.com.origin.snapshots.Expect;
import au.com.origin.snapshots.junit5.SnapshotExtension;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilTest;
import frc.robot.Constants.Pathing;
import frc.util.pathing.DisplayFieldArray.FieldSquare;
import org.junit.jupiter.api.extension.ExtendWith;

@ExtendWith({SnapshotExtension.class})
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

  private static FieldSquare[][] placeObstructions() {
    FieldSquare[][] fieldSquares = new FieldSquare[Pathing.CELL_X_MAX][Pathing.CELL_Y_MAX];

    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {
        final double xCoord = x * Pathing.CELL_SIZE_METERS;
        final double yCoord = y * Pathing.CELL_SIZE_METERS;
        fieldSquares[x][y] =
            FieldObstructionMap.isInsideObstruction(new Translation2d(xCoord, yCoord))
                ? FieldSquare.OBSTRUCTION
                : FieldSquare.EMPTY;
      }
    }

    return fieldSquares;
  }

  private Expect expect;

  @UtilTest
  public void trajectoryStateInversionMatchesSnapshot() {
    var fieldSquares = placeObstructions();
    var path = PathPlanner.loadPath("n2 engage", new PathConstraints(4, 2));

    var states = path.getStates();

    for (var state : states) {
      {
        final var inverted = PoseInversionUtility.findRedState(state);
        final var x =
            (int) (inverted.poseMeters.getTranslation().getX() / Pathing.CELL_SIZE_METERS);
        final var y =
            (int) (inverted.poseMeters.getTranslation().getY() / Pathing.CELL_SIZE_METERS);
        fieldSquares[x][y] = FieldSquare.PATH;
      }

      // also write the original pose to the field
      {
        final var x = (int) (state.poseMeters.getTranslation().getX() / Pathing.CELL_SIZE_METERS);
        final var y = (int) (state.poseMeters.getTranslation().getY() / Pathing.CELL_SIZE_METERS);
        fieldSquares[x][y] = FieldSquare.PATH;
      }
    }

    StringBuilder sb = new StringBuilder();
    DisplayFieldArray.renderField(sb, fieldSquares);

    expect.toMatchSnapshot(sb);
  }
}
