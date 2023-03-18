package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import au.com.origin.snapshots.Expect;
import au.com.origin.snapshots.junit5.SnapshotExtension;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilTest;
import frc.robot.Constants.Pathing;
import frc.util.pathing.DisplayFieldArray.FieldSquare;
import frc.util.pathing.FieldObstructionMap.RectangleObstruction;
import org.junit.jupiter.api.extension.ExtendWith;

@ExtendWith({SnapshotExtension.class})
public class FieldObstructionMapTests {

  @UtilTest
  public void rectangleContainsWorks() {
    RectangleObstruction rect =
        new RectangleObstruction(
            FieldObstructionMap.AllianceColor.RED,
            "test",
            new Translation2d(0, 0),
            new Translation2d(1, 1));

    assertTrue(
        rect.contains(new Translation2d(0, 0)), "Point (0, 0) should be in rect (0, 0) -> (1, 1)");
    assertTrue(
        rect.contains(new Translation2d(1, 1)), "Point (1, 1) should be in rect (0, 0) -> (1, 1)");
    assertTrue(
        rect.contains(new Translation2d(0.5, 0.5)),
        "Point (0.5, 0.5) should be in rect (0, 0) -> (1, 1)");

    assertFalse(
        rect.contains(new Translation2d(1.1, 1.1)),
        "Point (1.1, 1.1) should not be in rect (0, 0) -> (1, 1)");
  }

  @UtilTest
  public void obstructionMapMatchesSnapshot(Expect expect) {

    FieldSquare[][] obstructionMap = new FieldSquare[Pathing.CELL_X_MAX][Pathing.CELL_Y_MAX];

    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {
        final Translation2d point =
            new Translation2d(x * Pathing.CELL_SIZE_METERS, y * Pathing.CELL_SIZE_METERS);
        obstructionMap[x][y] =
            FieldObstructionMap.isInsideObstruction(point)
                ? FieldSquare.OBSTRUCTION
                : switch (FieldObstructionMap.getPriorityFlow(point)) {
                  case X_AXIS_PREFERRED -> FieldSquare.X_AXIS_FLOW;
                  case Y_AXIS_PREFERRED -> FieldSquare.Y_AXIS_FLOW;
                  case NO_PREFERENCE -> FieldSquare.EMPTY;
                };
      }
    }

    StringBuilder sb = new StringBuilder();
    sb.append(
        String.format(
            "Field length: %f, height: %f\n",
            FieldObstructionMap.FIELD_LENGTH, FieldObstructionMap.FIELD_HEIGHT));
    sb.append(
        String.format(
            "Pathing.CELL_X_MAX: %d, Pathing.CELL_Y_MAX: %d, Step Size: %f\n",
            Pathing.CELL_X_MAX, Pathing.CELL_Y_MAX, Pathing.CELL_SIZE_METERS));

    DisplayFieldArray.renderField(sb, obstructionMap);

    // add the list of obstructions
    sb.append("\n---\n\n");
    ObjectMapper mapper = new ObjectMapper();
    mapper.enable(SerializationFeature.INDENT_OUTPUT);
    for (FieldObstructionMap.Obstruction obstruction : FieldObstructionMap.obstructions) {
      try {
        sb.append(mapper.writeValueAsString(obstruction));
      } catch (Exception e) {
        sb.append("Error writing obstruction to string: ");
        sb.append(obstruction.getName());
      }
    }

    for (FieldObstructionMap.PriorityFlow flow : FieldObstructionMap.priorityFlows) {
      try {
        sb.append(mapper.writeValueAsString(flow));
      } catch (Exception e) {
        sb.append("Error writing flow to string: ");
        sb.append(flow.getName());
      }
    }

    expect.toMatchSnapshot(sb.toString());
  }
}
