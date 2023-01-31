package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.UtilTest;
import frc.util.Graph;
import java.util.List;
import java.util.Optional;

public class GraphPathfinderTests {
  private static GridCoord coord(int x, int y) {
    return new GridCoord(x, y);
  }

  private static void addCoords(Graph<GridCoord> graph, int xMax, int yMax) {
    for (int x = 0; x <= xMax; x++) {
      for (int y = 0; y <= yMax; y++) {
        graph.addNode(coord(x, y));
      }
    }
  }

  @UtilTest
  public void pathfinderFindsBasicPath() {
    Graph<GridCoord> graph = new Graph<>();
    addCoords(graph, 3, 3);

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

  @UtilTest
  public void pathfinderReturnsEmpty() {
    Graph<GridCoord> graph = new Graph<>();
    addCoords(graph, 5, 5);

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
  public void pathfinderPrefersDiagonalsIfCheaper() {
    Graph<GridCoord> graph = new Graph<>();
    addCoords(graph, 3, 3);

    // add edges such that diagonal is the same cost as horizontal/vertical
    for (int x = 0; x < 3; x++) {
      for (int y = 0; y < 3; y++) {
        graph.addEdge(coord(x, y), coord(x + 1, y), 1);
        graph.addEdge(coord(x, y), coord(x, y + 1), 1);
        graph.addEdge(coord(x, y), coord(x + 1, y + 1), 1);
      }
    }

    assertEquals(
        Optional.of(List.of(coord(0, 0), coord(1, 1), coord(2, 2), coord(3, 3))),
        GraphPathfinder.findPath(graph, coord(0, 0), coord(3, 3)));
  }

  @UtilTest
  public void pathfinderPrefersCardinalsIfCheaper() {
    Graph<GridCoord> graph = new Graph<>();
    addCoords(graph, 3, 3);

    // add edges such that diagonal is 2.5x the cost of horizontal/vertical
    for (int x = 0; x <= 3; x++) {
      for (int y = 0; y <= 3; y++) {
        if (x < 3) graph.addEdge(coord(x, y), coord(x + 1, y), 1);
        if (y < 3) graph.addEdge(coord(x, y), coord(x, y + 1), 1);
        if (x < 3 && y < 3) graph.addEdge(coord(x, y), coord(x + 1, y + 1), 5);
      }
    }

    var expectedPath =
        List.of(
            coord(0, 0),
            coord(1, 0),
            coord(2, 0),
            coord(3, 0),
            coord(3, 1),
            coord(3, 2),
            coord(3, 3));

    // ensure the path is available
    for (int i = 0; i < expectedPath.size() - 1; i++) {
      var from = expectedPath.get(i);
      var to = expectedPath.get(i + 1);
      assertTrue(graph.hasEdge(from, to), "Graph should have edge from " + from + " to " + to);
    }

    assertEquals(
        Optional.of(expectedPath), GraphPathfinder.findPath(graph, coord(0, 0), coord(3, 3)));
  }
}
