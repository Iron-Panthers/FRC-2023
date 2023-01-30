package frc.util.pathing;

import au.com.origin.snapshots.Expect;
import au.com.origin.snapshots.junit5.SnapshotExtension;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilTest;
import org.junit.jupiter.api.extension.ExtendWith;

@ExtendWith({SnapshotExtension.class})
public class FieldObstructionMapTests {

  @UtilTest
  public void obstructionMapMatchesSnapshot(Expect expect) {

    final double stepSize = 0.1;
    final int xMax = (int) Math.ceil(FieldObstructionMap.FIELD_WIDTH / stepSize);
    final int yMax = (int) Math.ceil(FieldObstructionMap.FIELD_LENGTH / stepSize);

    boolean[] obstructionMap = new boolean[(xMax * yMax)];

    for (int x = 0; x < xMax; x++) {
      for (int y = 0; y < yMax; y++) {
        final double xCoord = x * stepSize;
        final double yCoord = y * stepSize;
        obstructionMap[x + (y * xMax)] =
            FieldObstructionMap.isInsideObstruction(new Translation2d(xCoord, yCoord));
      }
    }

    StringBuilder sb = new StringBuilder();
    final int tickDistance = (int) Math.ceil(1d / stepSize);
    for (int x = -1; x < xMax; x++) {
      for (int y = -1; y < yMax; y++) {
        if (x == -1 || y == -1) {
          if (x == -1 && y % tickDistance == 0) {
            sb.append("|");
          } else if (y == -1 && x % tickDistance == 0) {
            sb.append("―");
          } else {
            sb.append(" ");
          }
        } else {
          sb.append(obstructionMap[x + (y * xMax)] ? "#" : ".");
        }
      }
      sb.append("\n");
    }

    // add the list of obstructions
    sb.append("\n\n");
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

    expect.toMatchSnapshot(sb.toString());
  }
}
