package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilTest;
import frc.util.Graph;
import java.util.List;
import java.util.Optional;

public class GraphPathfinderTests {
  private static Translation2d coord(double x, double y) {
    return new Translation2d(x, y);
  }

  @UtilTest
  public void pathfinderFindsPath() {
    Graph<Translation2d> graph = new Graph<>();
    graph.addEdge(coord(0, 0), coord(1, 0), 1);
    graph.addEdge(coord(1, 0), coord(2, 0), 1);
    graph.addEdge(coord(2, 0), coord(3, 0), 1);

    graph.addEdge(coord(0, 0), coord(0, 1), 1);
    graph.addEdge(coord(0, 1), coord(0, 2), 1);
    graph.addEdge(coord(0, 2), coord(0, 3), 1);

    graph.addEdge(coord(0, 0), coord(1, 1), 1.414);
    graph.addEdge(coord(1, 1), coord(2, 2), 1.414);

    assertEquals(
        Optional.of(List.of(coord(0, 0), coord(1, 0), coord(2, 0), coord(3, 0))),
        GraphPathfinder.findPath(graph, coord(0, 0), coord(3, 0)));
  }
}