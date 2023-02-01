package frc.util.pathing;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Pathing;
import frc.util.Graph;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class RubenManueverGenerator {
  private final Graph<GridCoord> adjacencyGraph = new Graph<>();

  private final boolean[][] collisionGrid = new boolean[Pathing.CELL_X_MAX][Pathing.CELL_Y_MAX];

  private static final double DIAGONAL_COST = Math.sqrt(2) + .001;

  /**
   * Determine if a given coordinate is valid for the pathing grid. Factors the robot's width into
   * consideration.
   *
   * @param coord The coordinate to check.
   * @return True if the coordinate is valid, false otherwise.
   */
  private boolean isValidCoord(GridCoord coord) {
    return coord.x >= 0
        && coord.x < Pathing.CELL_X_MAX
        && coord.y >= 0
        && coord.y < Pathing.CELL_Y_MAX
        && !collisionGrid[coord.x][coord.y];
  }

  private void addEdgeIfEndAccessible(GridCoord start, GridCoord end, double weight) {
    if (isValidCoord(end)) {
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

    // populate the collision grid
    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {

        // if the cell is inside an obstruction, mark it as a collision
        if (FieldObstructionMap.isInsideObstruction(new GridCoord(x, y).toTranslation2d())) {
          collisionGrid[x][y] = true;
          continue;
        }

        // if the cell would would intersect the robots radius, mark it as a collision
        var robotTopRight =
            new GridCoord(
                x + Pathing.ROBOT_RADIUS_UNDERESTIMATE_CELLS,
                y + Pathing.ROBOT_RADIUS_UNDERESTIMATE_CELLS);
        var robotBottomLeft =
            new GridCoord(
                x - Pathing.ROBOT_RADIUS_UNDERESTIMATE_CELLS,
                y - Pathing.ROBOT_RADIUS_UNDERESTIMATE_CELLS);

        boolean didIntersect = false;
        for (var obstruction : FieldObstructionMap.obstructions) {
          // rectangle intersection with robot body
          if (obstruction instanceof FieldObstructionMap.RectangleObstruction) {
            var rect = (FieldObstructionMap.RectangleObstruction) obstruction;
            var rectTopRight = new GridCoord(rect.topRight);
            var rectBottomLeft = new GridCoord(rect.bottomLeft);

            if (robotTopRight.x >= rectBottomLeft.x
                && robotTopRight.y >= rectBottomLeft.y
                && robotBottomLeft.x <= rectTopRight.x
                && robotBottomLeft.y <= rectTopRight.y) {
              collisionGrid[x][y] = true;
              didIntersect = true;
              break;
            }
          }
        }

        // the cell is not a collision, so allow it to be traversed
        if (!didIntersect) {
          collisionGrid[x][y] = false;
        }
      }
    }

    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {
        final GridCoord start = new GridCoord(x, y);

        if (!collisionGrid[x][y]) {
          adjacencyGraph.addNode(start);
        }
      }
    }

    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {
        final GridCoord start = new GridCoord(x, y);

        if (!collisionGrid[x][y]) {

          // Add edges to adjacent nodes
          for (GridCoord end : getOrthogonalTranslations(start)) {
            addEdgeIfEndAccessible(start, end, 1);
          }

          // Add edges to diagonal nodes
          for (GridCoord end : getDiagonalTranslations(start)) {
            addEdgeIfEndAccessible(start, end, DIAGONAL_COST);
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
    return !(
    // if the previous and next point are on the same line horizontal or vertical
    (prev.x == current.x && current.x == next.x)
        || (prev.y == current.y && current.y == next.y)
        // if the previous and next point are on the same line diagonal
        || (prev.x - current.x == prev.y - current.y && current.x - next.x == current.y - next.y)
        || (prev.x - current.x == current.y - prev.y && current.x - next.x == next.y - current.y));
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

  /*
  Where . is empty space, X is a point on the path, and * is a critical point:

  detect patterns that look like
  XX.
  ..X
  ..X

  with critical points
  X*.
  ..*
  ..X

  and simplify them to
  XX*
  ..X
  ..X

  additionally, detect the mirrored variants of this pattern, like
  X..
  *..
  .*x

  or

  X*.
  ..*
  ..X
  */
  private static boolean isCornerPattern(GridCoord p1, GridCoord p2) {
    // if the two points are adjacent diagonally
    return Math.abs(p1.x - p2.x) == 1 && Math.abs(p1.y - p2.y) == 1;
  }

  /**
   * Takes a list of critical points, and removes those that are less than a threshold from the line
   * between their previous and next point. Will always keep the first and last point.
   *
   * @param criticalPoints
   * @return The list of critical points with the unnecessary points removed.
   */
  public static List<GridCoord> simplifyCriticalPoints(List<GridCoord> criticalPoints) {
    return simplifyCriticalPoints(criticalPoints, 0);
  }

  private static List<GridCoord> simplifyCriticalPoints(List<GridCoord> criticalPoints, int pass) {
    if (pass > 2) return criticalPoints;

    List<GridCoord> simplifiedCriticalPoints = new ArrayList<>();

    if (criticalPoints.size() > 0) {
      simplifiedCriticalPoints.add(criticalPoints.get(0));
    }

    for (int i = 1; i < criticalPoints.size() - 1; i++) {
      GridCoord prev = criticalPoints.get(i - 1);
      GridCoord current = criticalPoints.get(i);
      GridCoord next = criticalPoints.get(i + 1);

      // if the current point does not diverge far enough from the line between the previous and
      // next
      // point, then it is not an important critical point
      if (Math.abs(
              (next.y - prev.y) * current.x
                  - (next.x - prev.x) * current.y
                  + next.x * prev.y
                  - next.y * prev.x)
          > Pathing.CRITICAL_POINT_DIVERGENCE_THRESHOLD) {
        simplifiedCriticalPoints.add(current);
      }
    }

    if (criticalPoints.size() > 1) {
      simplifiedCriticalPoints.add(criticalPoints.get(criticalPoints.size() - 1));
    }

    // there are no corner patterns if there are less than 3 points
    if (simplifiedCriticalPoints.size() < 3) return simplifiedCriticalPoints;

    List<GridCoord> simplifiedAndDeCorneredCriticalPoints = new ArrayList<>();

    // if the first two points match the pattern, keep the first point
    // if the last two points match the pattern, keep the last point
    // if two points match the pattern, keep the y of the first point and the x of the second point
    // if two points don't match the pattern, keep both points

    if (isCornerPattern(simplifiedCriticalPoints.get(0), simplifiedCriticalPoints.get(1))) {
      simplifiedAndDeCorneredCriticalPoints.add(simplifiedCriticalPoints.get(0));
    } else {
      simplifiedAndDeCorneredCriticalPoints.add(simplifiedCriticalPoints.get(0));
      simplifiedAndDeCorneredCriticalPoints.add(simplifiedCriticalPoints.get(1));
    }
    int i = 2;
    while (i < simplifiedCriticalPoints.size() - 2) {
      if (isCornerPattern(simplifiedCriticalPoints.get(i), simplifiedCriticalPoints.get(i + 1))) {
        simplifiedAndDeCorneredCriticalPoints.add(
            new GridCoord(
                simplifiedCriticalPoints.get(i).x, simplifiedCriticalPoints.get(i + 1).y));
        i += 2;
      } else {
        simplifiedAndDeCorneredCriticalPoints.add(simplifiedCriticalPoints.get(i));
        i++;
      }
    }
    if (isCornerPattern(
        simplifiedCriticalPoints.get(simplifiedCriticalPoints.size() - 2),
        simplifiedCriticalPoints.get(simplifiedCriticalPoints.size() - 1))) {
      simplifiedAndDeCorneredCriticalPoints.add(
          simplifiedCriticalPoints.get(simplifiedCriticalPoints.size() - 1));
    } else {
      simplifiedAndDeCorneredCriticalPoints.add(
          simplifiedCriticalPoints.get(simplifiedCriticalPoints.size() - 2));
      simplifiedAndDeCorneredCriticalPoints.add(
          simplifiedCriticalPoints.get(simplifiedCriticalPoints.size() - 1));
    }

    return simplifyCriticalPoints(simplifiedAndDeCorneredCriticalPoints, pass + 1);
  }

  private static Rotation2d straightLineAngle(Translation2d start, Translation2d end) {
    double x1 = start.getX();
    double y1 = start.getY();
    double x2 = end.getX();
    double y2 = end.getY();

    double angle = Math.atan2(y2 - y1, x2 - x1);
    return Rotation2d.fromRadians(angle);
  }

  public static List<PathPoint> computePathPointsFromCriticalPoints(
      List<GridCoord> criticalPoints) {
    List<PathPoint> pathPoints = new ArrayList<>();

    pathPoints.add(
        new PathPoint(
            // the position of the point
            criticalPoints.get(0).toTranslation2d(),
            // the heading of the spline, pointing towards the next point
            straightLineAngle(
                criticalPoints.get(0).toTranslation2d(), criticalPoints.get(1).toTranslation2d()),
            // the rotation of the robot
            // FIXME: interpolate this
            new Rotation2d()));

    for (int i = 1; i < criticalPoints.size() - 1; i++) {
      GridCoord prev = criticalPoints.get(i - 1);
      GridCoord current = criticalPoints.get(i);
      GridCoord next = criticalPoints.get(i + 1);

      pathPoints.add(
          new PathPoint(
              current.toTranslation2d(),
              // find the angle between the previous and next point to maintain smooth curvature
              straightLineAngle(prev.toTranslation2d(), next.toTranslation2d()),
              new Rotation2d()));
    }

    pathPoints.add(
        new PathPoint(
            criticalPoints.get(criticalPoints.size() - 1).toTranslation2d(),
            straightLineAngle(
                criticalPoints.get(criticalPoints.size() - 2).toTranslation2d(),
                criticalPoints.get(criticalPoints.size() - 1).toTranslation2d()),
            new Rotation2d()));

    return pathPoints;
  }
}
