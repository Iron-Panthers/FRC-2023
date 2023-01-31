package frc.util.pathing;

import frc.robot.Constants.Pathing;
import frc.util.Graph;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class RubenManueverGenerator {
  private final Graph<GridCoord> adjacencyGraph = new Graph<>();

  private static final double SQRT2 = Math.sqrt(2);

  private void addEdgeIfEndAccessible(GridCoord start, GridCoord end, double weight) {
    if (end.x >= 0
        && end.x < Pathing.CELL_X_MAX
        && end.y >= 0
        && end.y < Pathing.CELL_Y_MAX
        && !FieldObstructionMap.isInsideObstruction(start.toTranslation2d())
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

  private GridCoord[] getDiagonalTranslations(GridCoord start) {
    return new GridCoord[] {
      new GridCoord(start.x + 1, start.y + 1),
      new GridCoord(start.x - 1, start.y + 1),
      new GridCoord(start.x + 1, start.y - 1),
      new GridCoord(start.x - 1, start.y - 1)
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

          // Add edges to diagonal nodes
          for (GridCoord end : getDiagonalTranslations(start)) {
            addEdgeIfEndAccessible(start, end, SQRT2);
          }
        }
      }
    }

    adjacencyGraph.lock();
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

  public static boolean isCriticalPoint(GridCoord prev, GridCoord current, GridCoord next) {
    // if the previous and next point are on the same line horizontal or vertical
    if ((prev.x == current.x && current.x == next.x)
        || (prev.y == current.y && current.y == next.y)) return false;

    // if the previous and next point are on the same line diagonal
    if ((prev.x - current.x == prev.y - current.y && current.x - next.x == current.y - next.y)
        || (prev.x - current.x == current.y - prev.y && current.x - next.x == next.y - current.y))
      return false;

    return true;
  }

  /**
   * Finds the critical points in a path. Critical points are the points that demarcate a change in
   * direction. "Connecting the dots" of critical points would result in the same path as the
   * original.
   *
   * @param path The path to find the critical points of.
   * @return The list of critical points.
   */
  public static List<GridCoord> findCriticalPoints(List<GridCoord> path) {
    List<GridCoord> criticalPoints = new ArrayList<>();

    if (path.size() > 0) {
      criticalPoints.add(path.get(0));
    }

    for (int i = 1; i < path.size() - 1; i++) {
      GridCoord prev = path.get(i - 1);
      GridCoord current = path.get(i);
      GridCoord next = path.get(i + 1);

      if (isCriticalPoint(prev, current, next)) {
        criticalPoints.add(current);
      }
    }

    if (path.size() > 1) {
      criticalPoints.add(path.get(path.size() - 1));
    }

    return criticalPoints;
  }
}
