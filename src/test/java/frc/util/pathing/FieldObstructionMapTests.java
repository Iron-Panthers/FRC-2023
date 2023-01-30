package frc.util.pathing;

import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilTest;

public class FieldObstructionMapTests {

  @UtilTest
  public void obstructionMapMatchesString() {

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
    for (int x = 0; x < xMax; x++) {
      for (int y = 0; y < yMax; y++) {
        sb.append(obstructionMap[x + (y * xMax)] ? "██" : "  ");
      }
      sb.append("\n");
    }

    // System.out.println(sb.toString());
  }
}
