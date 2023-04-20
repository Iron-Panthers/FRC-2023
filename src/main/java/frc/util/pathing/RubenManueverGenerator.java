package frc.util.pathing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Pathing;
import frc.robot.Constants.Pathing.Costs;
import frc.util.Graph;
import frc.util.Util;
import frc.util.pathing.FieldObstructionMap.PriorityFlow.FlowType;
import frc.util.pathing.GridCoord.LinkDirection;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class RubenManueverGenerator {
  public final Graph<GridCoord> adjacencyGraph = new Graph<>();

  private final BoolGrid dangerGrid = new BoolGrid(Pathing.CELL_X_MAX, Pathing.CELL_Y_MAX);
  private final BoolGrid collisionGrid = new BoolGrid(Pathing.CELL_X_MAX, Pathing.CELL_Y_MAX);
  private final BoolGrid noPriorityFlowGrid = new BoolGrid(Pathing.CELL_X_MAX, Pathing.CELL_Y_MAX);

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
        && !collisionGrid.get(coord);
  }

  /**
   * Determine how much cost to multiply a given edge by based on the danger of the cell it leads to
   * and comes from.
   */
  private int computeDanger(GridCoord start, GridCoord end) {
    var danger = 1;

    if (dangerGrid.get(start)) danger *= Costs.DANGER_MULTIPLIER;

    if (dangerGrid.get(end)) danger *= Costs.DANGER_MULTIPLIER;

    return danger;
  }

  private int computeFlowPriority(GridCoord start, GridCoord end) {
    LinkDirection direction = GridCoord.getLinkDirection(start, end);
    FlowType flowTypeStart = FieldObstructionMap.getPriorityFlow(start.toTranslation2d());
    FlowType flowTypeEnd = FieldObstructionMap.getPriorityFlow(end.toTranslation2d());

    if (flowTypeStart == FlowType.NO_PREFERENCE && flowTypeEnd == FlowType.NO_PREFERENCE) {
      return 1;
    }

    if (direction == LinkDirection.COMBO) {
      return Costs.DIAGONAL_BAD_FLOW_PENALTY;
    }

    if (flowTypeStart == FlowType.X_AXIS_PREFERRED || flowTypeEnd == FlowType.X_AXIS_PREFERRED) {
      return direction == LinkDirection.PURE_X ? 1 : Costs.PERPENDICULAR_BAD_FLOW_PENALTY;
    }

    if (flowTypeStart == FlowType.Y_AXIS_PREFERRED || flowTypeEnd == FlowType.Y_AXIS_PREFERRED) {
      return direction == LinkDirection.PURE_Y ? 1 : Costs.PERPENDICULAR_BAD_FLOW_PENALTY;
    }

    // we should never actually get here
    return 1;
  }

  private void addEdgeIfEndAccessible(GridCoord start, GridCoord end, int weight) {
    if (isValidCoord(end)) {
      adjacencyGraph.addEdge(
          start, end, weight * computeDanger(start, end) * computeFlowPriority(start, end));
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

  private boolean robotCenterPointCollidesGivenWidth(int cellX, int cellY, int radius) {
    // if the cell would would intersect the robots radius, mark it as a collision
    var robotTopRight = new GridCoord(cellX + radius, cellY + radius);
    var robotBottomLeft = new GridCoord(cellX - radius, cellY - radius);

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
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Tool to find the critical spline points to go from {@link Pose2d} start to {@link Pose2d} end.
   * Construct this only once to reuse its adjacencyGraph.
   */
  public RubenManueverGenerator() {

    // populate the collision grid
    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {

        var t = new GridCoord(x, y).toTranslation2d();

        noPriorityFlowGrid.set(
            x, y, FieldObstructionMap.getPriorityFlow(t) == FlowType.NO_PREFERENCE);

        // if the cell is inside an obstruction, mark it as a collision
        if (FieldObstructionMap.isInsideObstruction(t)) {
          collisionGrid.set(x, y, true);
          dangerGrid.set(x, y, true);
          continue;
        }

        // if the cell would would intersect the robots collision radius, mark it as a collision
        if (robotCenterPointCollidesGivenWidth(x, y, Pathing.ROBOT_RADIUS_COLLISION_CELLS)) {
          collisionGrid.set(x, y, true);
          dangerGrid.set(x, y, true);
        } else if (robotCenterPointCollidesGivenWidth(x, y, Pathing.ROBOT_RADIUS_DANGER_CELLS)) {
          collisionGrid.set(x, y, false);
          dangerGrid.set(x, y, true);
        } else {
          // this is a safe place to put the center of the robot
          collisionGrid.set(x, y, false);
          dangerGrid.set(x, y, false);
        }
      }
    }

    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {
        if (collisionGrid.get(x, y)) continue;

        adjacencyGraph.addNode(new GridCoord(x, y));
      }
    }

    for (int x = 0; x < Pathing.CELL_X_MAX; x++) {
      for (int y = 0; y < Pathing.CELL_Y_MAX; y++) {
        if (collisionGrid.get(x, y)) continue;

        final GridCoord start = new GridCoord(x, y);

        // Add edges to adjacent nodes
        for (GridCoord end : getOrthogonalTranslations(start)) {
          addEdgeIfEndAccessible(start, end, Costs.CARDINAL);
        }

        // Add edges to diagonal nodes
        for (GridCoord end : getDiagonalTranslations(start)) {
          addEdgeIfEndAccessible(start, end, Costs.DIAGONAL);
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
    return GraphPathfinder.findPath(
        adjacencyGraph, start, end, Pathing.PATHFINDING_HEURISTIC_CONSTANT);
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
    if (path.size() < 3) {
      return path;
    }

    List<GridCoord> criticalPoints = new ArrayList<>();

    // find critical points by determining the furthest two points that are equivalent
    int startPosition = 0;
    int endPosition = 1;
    while (endPosition < path.size()) {
      var line = GridCoord.line(path.get(startPosition), path.get(endPosition));
      boolean isEquivalent = true;
      for (int i = 0; i < line.size(); i++) {
        if (!line.get(i).equals(path.get(startPosition + i))) {
          isEquivalent = false;
          break;
        }
      }
      if (isEquivalent) {
        endPosition++;
      } else {
        criticalPoints.add(path.get(startPosition));
        startPosition = endPosition - 1;
      }
    }
    criticalPoints.add(path.get(startPosition));
    criticalPoints.add(path.get(endPosition - 1));

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
   * Takes a list of critical points, and removes those don't contribute to the path, like corner
   * patterns and points that are almost a straight line.
   *
   * @param criticalPoints
   * @return The list of critical points with the unnecessary points removed.
   */
  public List<GridCoord> simplifyCriticalPoints(List<GridCoord> criticalPoints) {
    // cannot simplify a path with less than 3 points
    if (criticalPoints.size() <= 2) {
      return criticalPoints;
    }

    List<GridCoord> simplifiedCriticalPoints = new ArrayList<>();
    // remove corner patterns, replacing with a single point
    // never remove the first or last point--if they are part of a corner pattern, remove the point
    // that is not a start or end entirely
    for (int i = 0; i < criticalPoints.size() - 1; i++) {
      GridCoord p1 = criticalPoints.get(i);
      GridCoord p2 = criticalPoints.get(i + 1);

      if (isCornerPattern(p1, p2)) {
        if (criticalPoints.size() <= i + 2) {
          simplifiedCriticalPoints.add(p2);
        } else {
          // if the two points are adjacent diagonally
          GridCoord p3 = criticalPoints.get(i + 2);
          if (p1.x == p2.x) {
            // if the two points are horizontal
            if (p2.y == p3.y) {
              // if the third point is horizontal
              simplifiedCriticalPoints.add(new GridCoord(p1.x, p2.y));
            } else {
              // if the third point is vertical
              simplifiedCriticalPoints.add(new GridCoord(p2.x, p1.y));
            }
          } else {
            // if the two points are vertical
            if (p2.x == p3.x) {
              // if the third point is vertical
              simplifiedCriticalPoints.add(new GridCoord(p2.x, p1.y));
            } else {
              // if the third point is horizontal
              simplifiedCriticalPoints.add(new GridCoord(p1.x, p2.y));
            }
          }
          i++;
        }
      } else {
        simplifiedCriticalPoints.add(p1);
      }
    }
    if (simplifiedCriticalPoints.get(simplifiedCriticalPoints.size() - 1)
        != criticalPoints.get(criticalPoints.size() - 1)) {
      simplifiedCriticalPoints.add(criticalPoints.get(criticalPoints.size() - 1));
    }

    // remove points that are almost a straight line
    // this is done by finding a point that is within threshold distance of a line between the
    // previous and next point
    // and then making a new line between the previous and next point
    // and ensuring that new line does not have any points in the collision grid
    // if it does, then the point is not removed
    // if it does not, then the point is removed
    // this process is repeated until no points are removed

    // if the path is too short to remove any points, return the original path
    if (simplifiedCriticalPoints.size() <= 2) {
      return simplifiedCriticalPoints;
    }

    boolean removedPoint = true;
    while (removedPoint) {
      removedPoint = false;
      // we want to remove the points furthest from their neighbors first
      int bestCandidate = -1;
      double bestDistance = 0;
      for (int i = 1; i < simplifiedCriticalPoints.size() - 1; i++) {
        GridCoord p1 = simplifiedCriticalPoints.get(i - 1);
        GridCoord p2 = simplifiedCriticalPoints.get(i);
        GridCoord p3 = simplifiedCriticalPoints.get(i + 1);

        /** d = |ax_0 + by_0 + c| / sqrt(a^2 + b^2) */
        double a = p1.y - (double) p3.y;
        double b = p3.x - (double) p1.x;
        double c = p1.x * p3.y - (double) p3.x * p1.y;
        double d = Math.abs(a * p2.x + b * p2.y + c) / Math.sqrt(a * a + b * b);

        // if the point is within threshold distance of a line between the previous and next point
        // and greater than the best candidate so far
        if (d < Pathing.CRITICAL_POINT_DIVERGENCE_THRESHOLD && d > bestDistance) {
          // make a new line between the previous and next point
          List<GridCoord> line = GridCoord.line(p1, p3);
          // and ensure that new line does not have any points in the danger grid
          // and ensure that all points in the new line are in the same priority flow field
          boolean hasDangerOrPriorityFlow = false;
          for (GridCoord point : line) {
            if (dangerGrid.get(point) || !noPriorityFlowGrid.get(point)) {
              hasDangerOrPriorityFlow = true;
              break;
            }
          }

          if (hasDangerOrPriorityFlow) continue;

          // if the entire new segment is safe, then the point is our new best candidate to remove
          bestCandidate = i;
          bestDistance = d;
        }
      }
      // if we found a point to remove
      if (bestCandidate != -1) {
        // remove it
        simplifiedCriticalPoints.remove(bestCandidate);
        removedPoint = true;
      }
    }

    return simplifiedCriticalPoints;
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
      List<GridCoord> criticalPoints, Pose2d start, ChassisSpeeds chassisSpeeds, Pose2d end) {
    if (criticalPoints.size() < 2) {
      throw new IllegalArgumentException(
          "Cannot compute path with less than 2 critical points, as the first and last points are replaced with real poses.");
    }

    List<PathPoint> pathPoints = new ArrayList<>();

    pathPoints.add(
        Util.getVelocity(chassisSpeeds) > Pathing.RESPECT_CURRENT_VELOCITY_THRESHOLD_MS
            ? PathPoint.fromCurrentHolonomicState(start, chassisSpeeds)
            : new PathPoint(
                // the position of the point
                start.getTranslation(),
                // the heading of the spline, pointing towards the next point
                straightLineAngle(start.getTranslation(), criticalPoints.get(1).toTranslation2d()),
                // the rotation of the robot
                start.getRotation()));

    for (int i = 1; i < criticalPoints.size() - 1; i++) {
      GridCoord prev = criticalPoints.get(i - 1);
      GridCoord current = criticalPoints.get(i);
      GridCoord next = criticalPoints.get(i + 1);

      pathPoints.add(
          new PathPoint(
              current.toTranslation2d(),
              // find the angle between the previous and next point to maintain smooth curvature
              straightLineAngle(prev.toTranslation2d(), next.toTranslation2d()),
              // FIXME: interpolate this
              end.getRotation()));
    }

    pathPoints.add(
        new PathPoint(
            end.getTranslation(),
            straightLineAngle(
                criticalPoints.get(criticalPoints.size() - 2).toTranslation2d(),
                end.getTranslation()),
            end.getRotation()));

    return pathPoints;
  }

  public Optional<PathPlannerTrajectory> computePath(
      Pose2d start, ChassisSpeeds chassisSpeeds, Pose2d end, PathConstraints constraints) {

    Pose2d projectedStart = // check if we are moving fast enough to matter
        Util.getVelocity(chassisSpeeds) > Pathing.RESPECT_CURRENT_VELOCITY_THRESHOLD_MS
            ? new Pose2d(
                start
                    .getTranslation()
                    .plus(
                        Util.getTranslationVelocity(chassisSpeeds, start.getRotation())
                            .times(Pathing.ANTICIPATED_PATH_SOLVE_TIME_SECONDS)),
                start.getRotation())
            : start;

    // convert the start and end point to grid coordinates
    GridCoord startCoord = new GridCoord(projectedStart.getTranslation());
    GridCoord endCoord = new GridCoord(end.getTranslation());

    // System.out.println("start: " + new GridCoord(start.getTranslation()));
    // System.out.println("projected: " + startCoord);
    // System.out.println("chassis speeds: " + chassisSpeeds);
    // var t1 = Timer.getFPGATimestamp();
    var path = findFullPath(startCoord, endCoord);
    // System.out.println("path solve time: " + (Timer.getFPGATimestamp() - t1));
    if (path.isEmpty()) return Optional.empty();
    var criticalPoints = findCriticalPoints(path.get());
    var neededCriticalPoints = simplifyCriticalPoints(criticalPoints);
    var pathPoints =
        neededCriticalPoints.size() < 2
            ? List.of(
                new PathPoint(
                    start.getTranslation(),
                    straightLineAngle(start.getTranslation(), end.getTranslation()),
                    start.getRotation()),
                new PathPoint(
                    end.getTranslation(),
                    straightLineAngle(end.getTranslation(), start.getTranslation()),
                    end.getRotation()))
            : computePathPointsFromCriticalPoints(
                neededCriticalPoints, projectedStart, chassisSpeeds, end);

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(constraints, pathPoints);

    // System.out.println("work time: " + (Timer.getFPGATimestamp() - t1));

    return Optional.of(trajectory);
  }
}
