package frc.util.pathing;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Pathing;
import frc.util.Graph;

public class RubenManueverGenerator {
  private final Graph<Translation2d> adjacencyGraph = Graph.warnOnImplicitNodeKeyAdded();

  private void addEdgeIfEndAccessible(Translation2d start, Translation2d end, double weight) {
    if (end.getX() >= 0
        && end.getX() <= FieldObstructionMap.FIELD_LENGTH
        && end.getY() >= 0
        && end.getY() <= FieldObstructionMap.FIELD_HEIGHT
        && !FieldObstructionMap.isInsideObstruction(end)) {
      adjacencyGraph.addEdge(start, end, weight);
    }
  }

  private Translation2d[] getOrthogonalTranslations(Translation2d start) {
    return new Translation2d[] {
      new Translation2d(start.getX() + Pathing.CELL_SIZE_METERS, start.getY()),
      new Translation2d(start.getX() - Pathing.CELL_SIZE_METERS, start.getY()),
      new Translation2d(start.getX(), start.getY() + Pathing.CELL_SIZE_METERS),
      new Translation2d(start.getX(), start.getY() - Pathing.CELL_SIZE_METERS)
    };
  }

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
        final Translation2d start = new Translation2d(xCoord, yCoord);

        if (!FieldObstructionMap.isInsideObstruction(start)) {
          adjacencyGraph.addNode(start);

          // Add edges to adjacent nodes
          for (Translation2d end : getOrthogonalTranslations(new Translation2d(xCoord, yCoord))) {
            addEdgeIfEndAccessible(start, end, Pathing.CELL_SIZE_METERS);
          }

          // TODO: Add edges to diagonal nodes
        }
      }
    }
  }
}