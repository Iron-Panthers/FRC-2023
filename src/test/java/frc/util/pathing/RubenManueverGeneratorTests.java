package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import au.com.origin.snapshots.Expect;
import au.com.origin.snapshots.junit5.SnapshotExtension;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilParamTest;
import frc.UtilTest;
import frc.robot.Constants.Pathing;
import frc.util.Graph;
import frc.util.pathing.DisplayFieldArray.FieldSquare;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

@ExtendWith({SnapshotExtension.class})
public class RubenManueverGeneratorTests {

  @UtilTest
  public void constructsWithoutError() {
    assertDoesNotThrow(RubenManueverGenerator::new);
  }

  @UtilTest
  public void unobstructedCordsAreDoublyLinked() {
    RubenManueverGenerator rubenManueverGenerator = new RubenManueverGenerator();

    // use java reflection to access adjacencyGraph
    Optional<Graph<GridCoord>> optAdjacencyGraph = Optional.empty();
    try {
      Field field = RubenManueverGenerator.class.getDeclaredField("adjacencyGraph");
      field.setAccessible(true);

      var obj = field.get(rubenManueverGenerator);

      optAdjacencyGraph =
          obj instanceof Graph<?> ? Optional.of((Graph<GridCoord>) obj) : Optional.empty();
    } catch (NoSuchFieldException | IllegalAccessException e) {
      e.printStackTrace();
    }

    if (optAdjacencyGraph.isEmpty()) {
      fail("optAdjacencyGraph is empty because reflection failed");
    }

    var adjacencyGraph = optAdjacencyGraph.get();

    var coords =
        List.of(
            // basic cardinals
            Pair.of(new Translation2d(5, 5), new Translation2d(5, 5 + Pathing.CELL_SIZE_METERS)),
            Pair.of(new Translation2d(5, 5), new Translation2d(5 + Pathing.CELL_SIZE_METERS, 5)),
            Pair.of(new Translation2d(5, 5), new Translation2d(5, 5 - Pathing.CELL_SIZE_METERS)),
            Pair.of(new Translation2d(5, 5), new Translation2d(5 - Pathing.CELL_SIZE_METERS, 5)),
            // straight line connectivity
            Pair.of(
                new Translation2d(5 + Pathing.CELL_SIZE_METERS, 5),
                new Translation2d(5 + Pathing.CELL_SIZE_METERS * 2, 5))
            // Pair.of(
            //     coord(5 + Pathing.CELL_SIZE_METERS * 2, 5),
            //     coord(5 + Pathing.CELL_SIZE_METERS * 3, 5))
            // brace holder for autoformatting
            );

    for (var coord : coords) {
      var firstGridCoord = new GridCoord(coord.getFirst());
      var secondGridCoord = new GridCoord(coord.getSecond());
      assertFalse(
          FieldObstructionMap.isInsideObstruction(coord.getFirst()),
          String.format("coord %s is inside an obstruction", coord.getFirst()));
      assertFalse(
          FieldObstructionMap.isInsideObstruction(coord.getSecond()),
          String.format("coord %s is inside an obstruction", coord.getSecond()));
      assertTrue(
          adjacencyGraph.hasNode(firstGridCoord),
          String.format("Node %s is missing", firstGridCoord));
      assertTrue(
          adjacencyGraph.hasNode(secondGridCoord),
          String.format("Node %s is missing", secondGridCoord));
      assertTrue(
          adjacencyGraph.getNeighbors(firstGridCoord).contains(secondGridCoord),
          String.format("coord %s should be linked to %s", firstGridCoord, secondGridCoord));
      assertTrue(
          adjacencyGraph.getNeighbors(secondGridCoord).contains(firstGridCoord),
          String.format("coord %s should be linked back to %s", secondGridCoord, firstGridCoord));
    }
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

  public static Stream<Arguments> findFullPathMatchesSnapshotProvider() {
    return Stream.of(
        Arguments.of(new GridCoord(50, 50), new GridCoord(50, 55)),
        Arguments.of(new GridCoord(10, 62), new GridCoord(32, 10))
        // load bearing comment (hold the final brace)
        );
  }

  private Expect expect;

  @UtilParamTest
  @MethodSource("findFullPathMatchesSnapshotProvider")
  public void findFullPathMatchesSnapshot(GridCoord start, GridCoord end) {
    RubenManueverGenerator rubenManueverGenerator = new RubenManueverGenerator();

    FieldSquare[][] fieldSquares = placeObstructions();

    var path = rubenManueverGenerator.findFullPath(start, end);

    for (var coord : path.get()) {
      fieldSquares[coord.x][coord.y] = FieldSquare.PATH;
    }

    fieldSquares[start.x][start.y] = FieldSquare.START;
    fieldSquares[end.x][end.y] = FieldSquare.END;

    StringBuilder sb = new StringBuilder();
    DisplayFieldArray.renderField(sb, fieldSquares);

    expect
        .scenario(String.format("%s -> %s", start.toString(), end.toString()))
        .toMatchSnapshot(sb.toString());
  }

  public static Stream<Arguments> findPathAndCriticalPointsMatchesSnapshotProvider() {
    return Stream.of(
        Arguments.of(new GridCoord(50, 50), new GridCoord(50, 55)),
        Arguments.of(new GridCoord(10, 62), new GridCoord(32, 10))
        // load bearing comment (hold the final brace)
        );
  }

  @UtilParamTest
  @MethodSource("findPathAndCriticalPointsMatchesSnapshotProvider")
  public void findPathAndCriticalPointsMatchesSnapshot(GridCoord start, GridCoord end) {
    RubenManueverGenerator rubenManueverGenerator = new RubenManueverGenerator();

    FieldSquare[][] fieldSquares = placeObstructions();

    var path = rubenManueverGenerator.findFullPath(start, end);
    var criticalPoints = RubenManueverGenerator.findCriticalPoints(path.get());

    for (var coord : path.get()) {
      fieldSquares[coord.x][coord.y] = FieldSquare.PATH;
    }

    for (var coord : criticalPoints) {
      fieldSquares[coord.x][coord.y] = FieldSquare.CRITICAL_POINT;
    }

    // fieldSquares[start.x][start.y] = FieldSquare.START;
    // fieldSquares[end.x][end.y] = FieldSquare.END;

    StringBuilder sb = new StringBuilder();
    DisplayFieldArray.renderField(sb, fieldSquares);

    expect
        .scenario(
            String.format(
                "%s -> %s len: %s",
                start.toString(), end.toString(), path.map(List::size).orElse(-1)))
        .toMatchSnapshot(sb.toString());
  }

  public static Stream<Arguments> findRedundantCriticalPointsProvider() {
    return Stream.of(
        Arguments.of(new GridCoord(50, 50), new GridCoord(50, 55)),
        Arguments.of(new GridCoord(10, 62), new GridCoord(32, 10))
        // load bearing comment (hold the final brace)
        );
  }

  @UtilParamTest
  @MethodSource("findRedundantCriticalPointsProvider")
  public void findRedundantCriticalPoints(GridCoord start, GridCoord end) {
    RubenManueverGenerator rubenManueverGenerator = new RubenManueverGenerator();

    FieldSquare[][] fieldSquares = placeObstructions();

    var path = rubenManueverGenerator.findFullPath(start, end);
    var criticalPoints = RubenManueverGenerator.findCriticalPoints(path.get());
    var neededCriticalPoints = RubenManueverGenerator.simplifyCriticalPoints(criticalPoints);

    for (var coord : path.get()) {
      fieldSquares[coord.x][coord.y] = FieldSquare.PATH;
    }

    for (var coord : criticalPoints) {
      fieldSquares[coord.x][coord.y] = FieldSquare.REDUNDANT_CRITICAL_POINT;
    }

    for (var coord : neededCriticalPoints) {
      fieldSquares[coord.x][coord.y] = FieldSquare.CRITICAL_POINT;
    }

    // fieldSquares[start.x][start.y] = FieldSquare.START;
    // fieldSquares[end.x][end.y] = FieldSquare.END;

    StringBuilder sb = new StringBuilder();
    DisplayFieldArray.renderField(sb, fieldSquares);

    expect
        .scenario(
            String.format(
                "%s -> %s len: %s",
                start.toString(), end.toString(), path.map(List::size).orElse(-1)))
        .toMatchSnapshot(sb.toString());
  }

  public static Stream<Arguments> computePathPointsForSplineProvider() {
    return Stream.of(
        Arguments.of(new GridCoord(50, 50), new GridCoord(50, 55)),
        Arguments.of(new GridCoord(10, 62), new GridCoord(32, 10))
        // load bearing comment (hold the final brace)
        );
  }

  @UtilParamTest
  @MethodSource("computePathPointsForSplineProvider")
  public void computePathPointsForSplineMatchesSnapshot(GridCoord start, GridCoord end) {
    RubenManueverGenerator rubenManueverGenerator = new RubenManueverGenerator();

    FieldSquare[][] fieldSquares = placeObstructions();

    var path = rubenManueverGenerator.findFullPath(start, end);
    var criticalPoints = RubenManueverGenerator.findCriticalPoints(path.get());
    var pathPoints = RubenManueverGenerator.computePathPointsFromCriticalPoints(criticalPoints);

    for (var coord : path.get()) {
      fieldSquares[coord.x][coord.y] = FieldSquare.PATH;
    }

    // build a spline
    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(new PathConstraints(3, 1), pathPoints);

    // draw the spline
    for (var states : trajectory.getStates()) {
      var coord = new GridCoord(states.poseMeters.getTranslation());
      fieldSquares[coord.x][coord.y] = FieldSquare.SPLINE;
    }

    for (var coord : criticalPoints) {
      fieldSquares[coord.x][coord.y] = FieldSquare.CRITICAL_POINT;
    }

    // fieldSquares[start.x][start.y] = FieldSquare.START;
    // fieldSquares[end.x][end.y] = FieldSquare.END;

    StringBuilder sb = new StringBuilder();
    DisplayFieldArray.renderField(sb, fieldSquares);

    // write the pathpoints to the sb
    for (var point : pathPoints) {
      // use reflection to set the access and get the position, heading, and holonomic rotation
      try {
        Field positionField = point.getClass().getDeclaredField("position");
        positionField.setAccessible(true);
        Translation2d position = (Translation2d) positionField.get(point);

        Field headingField = point.getClass().getDeclaredField("heading");
        headingField.setAccessible(true);
        Rotation2d heading = (Rotation2d) headingField.get(point);

        Field holonomicRotationField = point.getClass().getDeclaredField("holonomicRotation");
        holonomicRotationField.setAccessible(true);
        Rotation2d holonomicRotation = (Rotation2d) holonomicRotationField.get(point);

        sb.append(
            String.format("pos:%s heading:%s rotation:%s\n", position, heading, holonomicRotation));
      } catch (NoSuchFieldException | IllegalAccessException e) {
        sb.append("error getting field\n");
      }
    }

    expect
        .scenario(
            String.format(
                "%s -> %s len: %s",
                start.toString(), end.toString(), path.map(List::size).orElse(-1)))
        .toMatchSnapshot(sb.toString());
  }
}
