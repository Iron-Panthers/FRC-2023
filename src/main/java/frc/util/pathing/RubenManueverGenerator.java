package frc.util.pathing;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Pathing;
import frc.util.Graph;

public class RubenManueverGenerator {
  private final Graph<Translation2d> adjacencyGraph = Graph.warnOnImplicitNodeKeyAdded();

  /**
   * Finds the critical spline points to go from {@link Pose2d} start to {@link Pose2d} end.
   * Construct this only once to reuse its adjacencyGraph.
   */
  public RubenManueverGenerator() {
    final int xMax = (int) Math.ceil(FieldObstructionMap.FIELD_LENGTH / Pathing.CELL_SIZE_METERS);
    final int yMax = (int) Math.ceil(FieldObstructionMap.FIELD_HEIGHT / Pathing.CELL_SIZE_METERS);

    for (int x = 0; x < xMax; x++) {
      for (int y = 0; y < yMax; y++) {
        final double xCoord = x * Pathing.CELL_SIZE_METERS;
        final double yCoord = y * Pathing.CELL_SIZE_METERS;
        if (!FieldObstructionMap.isInsideObstruction(new Translation2d(xCoord, yCoord))) {
          adjacencyGraph.addNode(new Translation2d(xCoord, yCoord));
        }
      }
    }
  }
}
