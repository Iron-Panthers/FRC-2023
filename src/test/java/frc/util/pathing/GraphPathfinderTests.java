package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.UtilTest;
import frc.util.Graph;
import java.util.List;
import java.util.Optional;

public class GraphPathfinderTests {
  private static GridCoord coord(int x, int y) {
    return new GridCoord(x, y);
  }

  @UtilTest
  public void pathfinderFindsBasicPath() {
    Graph<GridCoord> graph = new Graph<>();
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

    assertEquals(
        Optional.of(List.of(coord(0, 0), coord(1, 0), coord(2, 0), coord(3, 0))),
        GraphPathfinder.findPath(graph, coord(0, 0), coord(3, 0)));
  }

  @UtilTest
  public void pathfinderFindsCheaperEdgePath() {
    Graph<GridCoord> graph = new Graph<>();
    graph.addEdge(coord(0, 0), coord(1, 0), 1);
    graph.addEdge(coord(1, 0), coord(2, 0), 1);
    graph.addEdge(coord(2, 0), coord(3, 0), 1);
    graph.addEdge(coord(3, 0), coord(4, 0), 1);
    graph.addEdge(coord(4, 0), coord(5, 0), 1);

    graph.addEdge(coord(0, 0), coord(0, 1), 4);
    graph.addEdge(coord(0, 1), coord(0, 2), 4);

    // teleport!
    graph.addEdge(coord(5, 0), coord(0, 2), 2);

    assertEquals(
        Optional.of(
            List.of(
                coord(0, 0),
                coord(1, 0),
                coord(2, 0),
                coord(3, 0),
                coord(4, 0),
                coord(5, 0),
                coord(0, 2))),
        GraphPathfinder.findPath(graph, coord(0, 0), coord(0, 2)));
  }

  @UtilTest
  public void pathfinderReturnsEmpty() {
    Graph<GridCoord> graph = new Graph<>();
    graph.addEdge(coord(0, 0), coord(1, 0), 1);
    graph.addEdge(coord(1, 0), coord(2, 0), 1);
    graph.addEdge(coord(2, 0), coord(3, 0), 1);
    graph.addEdge(coord(3, 0), coord(4, 0), 1);
    graph.addEdge(coord(4, 0), coord(5, 0), 1);

    graph.addEdge(coord(0, 0), coord(0, 1), 4);
    graph.addEdge(coord(0, 1), coord(0, 2), 4);

    // teleport!
    graph.addEdge(coord(5, 0), coord(0, 2), 2);

    assertEquals(Optional.empty(), GraphPathfinder.findPath(graph, coord(0, 0), coord(0, 3)));
  }

  @UtilTest
  public void pathfinderWorksWithWeightSmallerThanOne() {
    Graph<GridCoord> graph = new Graph<>();
    graph.addEdge(coord(0, 0), coord(1, 0), .1);
    graph.addEdge(coord(1, 0), coord(2, 0), .1);
    graph.addEdge(coord(2, 0), coord(3, 0), .1);
    graph.addEdge(coord(3, 0), coord(4, 0), .1);
    graph.addEdge(coord(4, 0), coord(5, 0), .1);

    graph.addEdge(coord(0, 0), coord(0, 1), .4);
    graph.addEdge(coord(0, 1), coord(0, 2), .4);

    // teleport!
    graph.addEdge(coord(5, 0), coord(0, 2), .2);

    assertEquals(
        Optional.of(
            List.of(
                coord(0, 0),
                coord(1, 0),
                coord(2, 0),
                coord(3, 0),
                coord(4, 0),
                coord(5, 0),
                coord(0, 2))),
        GraphPathfinder.findPath(graph, coord(0, 0), coord(0, 2)));
  }
}
