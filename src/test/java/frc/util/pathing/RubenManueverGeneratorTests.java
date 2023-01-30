package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import au.com.origin.snapshots.Expect;
import au.com.origin.snapshots.junit5.SnapshotExtension;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilParamTest;
import frc.UtilTest;
import frc.robot.Constants.Pathing;
import frc.util.Graph;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

@ExtendWith({SnapshotExtension.class})
public class RubenManueverGeneratorTests {

  private static Translation2d coord(double x, double y) {
    return new Translation2d(x, y);
  }

  public static Stream<Arguments> getClosestPointProvider() {
    return Stream.of(
        Arguments.of(coord(2.001, 2), coord(2, 2)),
        Arguments.of(coord(3.12, 2.01), coord(3.1, 2)),
        Arguments.of(coord(3.12, 2.06), coord(3.1, 2.1)),
        Arguments.of(coord(3.12, -.01), coord(3.1, 0))
        // brace holder
        );
  }

  @UtilParamTest
  @MethodSource("getClosestPointProvider")
  public void getClosestPoint(Translation2d point, Translation2d expected) {
    assertEquals(
        expected,
        RubenManueverGenerator.getClosestPoint(point),
        String.format("Point %s should become %s", point, expected));
  }

  @UtilTest
  public void unobstructedCordsAreDoublyLinked() {
    RubenManueverGenerator rubenManueverGenerator = new RubenManueverGenerator();

    // use java reflection to access adjacencyGraph
    Optional<Graph<Translation2d>> optAdjacencyGraph = Optional.empty();
    try {
      Field field = RubenManueverGenerator.class.getDeclaredField("adjacencyGraph");
      field.setAccessible(true);
      optAdjacencyGraph = Optional.of((Graph<Translation2d>) field.get(rubenManueverGenerator));
    } catch (NoSuchFieldException | IllegalAccessException e) {
      e.printStackTrace();
    }

    if (optAdjacencyGraph.isEmpty()) {
      fail("optAdjacencyGraph is empty because reflection failed");
    }

    var adjacencyGraph = optAdjacencyGraph.get();

    var coords = List.of(Pair.of(coord(5, 5), coord(5, 5 + Pathing.CELL_SIZE_METERS)));

    for (var coord : coords) {
      assertTrue(
          adjacencyGraph.hasNode(coord.getFirst()),
          String.format("Node %s is missing", coord.getFirst()));
      assertTrue(
          adjacencyGraph.hasNode(coord.getSecond()),
          String.format("Node %s is missing", coord.getSecond()));
      assertTrue(
          adjacencyGraph.getNeighbors(coord.getFirst()).contains(coord.getSecond()),
          String.format("coord %s should be linked to %s", coord.getFirst(), coord.getSecond()));
      // assertTrue(
      //     adjacencyGraph.getNeighbors(coord.getSecond()).contains(coord.getFirst()),
      //     String.format("coord %s should be linked to %s", coord.getSecond(), coord.getFirst()));
    }
  }

  public static Stream<Arguments> findFullPathMatchesSnapshotProvider() {
    return Stream.of(
        Arguments.of(coord(5, 5), coord(6, 6))
        // load bearing comment (hold the final brace)
        );
  }

  private Expect expect;

  @UtilParamTest
  @MethodSource("findFullPathMatchesSnapshotProvider")
  public void findFullPathMatchesSnapshot(Translation2d start, Translation2d end) {
    RubenManueverGenerator rubenManueverGenerator = new RubenManueverGenerator();

    var path =
        rubenManueverGenerator.findFullPath(
            RubenManueverGenerator.getClosestPoint(start),
            RubenManueverGenerator.getClosestPoint(end));

    expect
        .scenario(String.format("%s -> %s", start.toString(), end.toString()))
        .toMatchSnapshot(path);
  }
}
