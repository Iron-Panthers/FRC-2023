package frc.util.pathing;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Pathing;
import frc.util.Graph;
import java.util.List;
import java.util.Optional;

public class RubenManueverGenerator {
  private final Graph<GridCoord> adjacencyGraph = new Graph<>();

  private void addEdgeIfEndAccessible(GridCoord start, GridCoord end, double weight) {
    if (end.x >= 0
        && end.x <= Pathing.CELL_X_MAX
        && end.y >= 0
        && end.y <= Pathing.CELL_Y_MAX
        && !FieldObstructionMap.isInsideObstruction(end.toTranslation2d())) {
      adjacencyGraph.addEdge(start, end, weight);
    }
  }

  private GridCoord[] getOrthogonalTranslations(GridCoord start) {
    return new GridCoord[] {
      new GridCoord(start.x + 1, start.y),
      new GridCoord(start.x - 1, start.y),
      new GridCoord(start.x, start.y + 1),
      new GridCoord(start.x, start.y - 1)
    };
  }

  /**
   * Tool to find the critical spline points to go from {@link Pose2d} start to {@link Pose2d} end.
   * Construct this only once to reuse its adjacencyGraph.
   */
  public RubenManueverGenerator() {

    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {
        final GridCoord start = new GridCoord(x, y);

        if (!FieldObstructionMap.isInsideObstruction(start.toTranslation2d())) {
          adjacencyGraph.addNode(start);
        }
      }
    }

    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {
        final GridCoord start = new GridCoord(x, y);

        if (!FieldObstructionMap.isInsideObstruction(start.toTranslation2d())) {

          // Add edges to adjacent nodes
          for (GridCoord end : getOrthogonalTranslations(start)) {
            addEdgeIfEndAccessible(start, end, 1);
          }

          // TODO: Add edges to diagonal nodes
        }
      }
    }

    adjacencyGraph.lock();
  }

  /**
   * Find the translation2d that is on the cell grid, by rounding the x and y coordinates to the
   * nearest cell size.
   *
   * @param point The point to round to the nearest cell.
   * @return The rounded point.
   */
  public static Translation2d getClosestPoint(Translation2d point) {
    return new Translation2d(
        Math.round(point.getX() / Pathing.CELL_SIZE_METERS) * Pathing.CELL_SIZE_METERS,
        Math.round(point.getY() / Pathing.CELL_SIZE_METERS) * Pathing.CELL_SIZE_METERS);
  }

  /**
   * Find the full set of grid coords on the cell grid that are between the start and end points.
   *
   * @param start The start point.
   * @param end The end point.
   * @return The list of grid coords between the start and end points.
   */
  public Optional<List<GridCoord>> findFullPath(GridCoord start, GridCoord end) {
    return GraphPathfinder.findPath(adjacencyGraph, start, end);
  }
}
