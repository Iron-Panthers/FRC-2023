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
    graph.addNode(coord(0, 0));
    graph.addNode(coord(1, 0));
    graph.addNode(coord(2, 0));
    graph.addNode(coord(3, 0));
    graph.addNode(coord(4, 0));
    graph.addNode(coord(5, 0));
    graph.addNode(coord(0, 1));
    graph.addNode(coord(0, 2));
    graph.addNode(coord(0, 3));
    graph.addNode(coord(0, 4));
    graph.addNode(coord(0, 5));
    graph.addNode(coord(1, 1));
    graph.addNode(coord(2, 2));
    graph.addNode(coord(3, 3));
    graph.addNode(coord(4, 4));
    graph.addNode(coord(5, 5));

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
    graph.addNode(coord(0, 0));
    graph.addNode(coord(1, 0));
    graph.addNode(coord(2, 0));
    graph.addNode(coord(3, 0));
    graph.addNode(coord(4, 0));
    graph.addNode(coord(5, 0));
    graph.addNode(coord(0, 1));
    graph.addNode(coord(0, 2));
    graph.addNode(coord(0, 3));
    graph.addNode(coord(0, 4));
    graph.addNode(coord(0, 5));
    graph.addNode(coord(1, 1));
    graph.addNode(coord(2, 2));
    graph.addNode(coord(3, 3));
    graph.addNode(coord(4, 4));
    graph.addNode(coord(5, 5));

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
    graph.addNode(coord(0, 0));
    graph.addNode(coord(1, 0));
    graph.addNode(coord(2, 0));
    graph.addNode(coord(3, 0));
    graph.addNode(coord(4, 0));
    graph.addNode(coord(5, 0));
    graph.addNode(coord(0, 1));
    graph.addNode(coord(0, 2));
    graph.addNode(coord(0, 3));
    graph.addNode(coord(0, 4));
    graph.addNode(coord(0, 5));
    graph.addNode(coord(1, 1));
    graph.addNode(coord(2, 2));
    graph.addNode(coord(3, 3));
    graph.addNode(coord(4, 4));
    graph.addNode(coord(5, 5));

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
    // define the nodes
    graph.addNode(coord(0, 0));
    graph.addNode(coord(1, 0));
    graph.addNode(coord(2, 0));
    graph.addNode(coord(3, 0));
    graph.addNode(coord(4, 0));
    graph.addNode(coord(5, 0));
    graph.addNode(coord(0, 1));
    graph.addNode(coord(0, 2));
    graph.addNode(coord(0, 3));
    graph.addNode(coord(0, 4));
    graph.addNode(coord(0, 5));
    graph.addNode(coord(1, 1));
    graph.addNode(coord(2, 2));
    graph.addNode(coord(3, 3));
    graph.addNode(coord(4, 4));
    graph.addNode(coord(5, 5));

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

  @UtilTest
  public void pathfinderFindsShortestPathOnGrid() {
    Graph<GridCoord> graph = new Graph<>();
    for (int x = 0; x < 20; x++) {
      for (int y = 0; y < 20; y++) {
        graph.addNode(coord(x, y));
      }
    }

    for (int x = 0; x < 20; x++) {
      for (int y = 0; y < 19; y++) {
        graph.addEdge(coord(x, y), coord(x, y + 1), 1);
      }
    }

    for (int x = 0; x < 19; x++) {
      for (int y = 0; y < 20; y++) {
        graph.addEdge(coord(x, y), coord(x + 1, y), 1);
      }
    }

    assertEquals(
        Optional.of(
            List.of(
                coord(0, 0),
                coord(1, 0),
                coord(2, 0),
                coord(3, 0),
                coord(4, 0),
                coord(5, 0),
                coord(6, 0),
                coord(7, 0),
                coord(8, 0),
                coord(9, 0),
                coord(10, 0),
                coord(11, 0),
                coord(12, 0),
                coord(13, 0),
                coord(14, 0),
                coord(15, 0),
                coord(16, 0),
                coord(17, 0),
                coord(18, 0),
                coord(19, 0),
                coord(19, 1),
                coord(19, 2),
                coord(19, 3),
                coord(19, 4),
                coord(19, 5),
                coord(19, 6),
                coord(19, 7),
                coord(19, 8),
                coord(19, 9),
                coord(19, 10),
                coord(19, 11),
                coord(19, 12),
                coord(19, 13),
                coord(19, 14),
                coord(19, 15),
                coord(19, 16),
                coord(19, 17),
                coord(19, 18),
                coord(19, 19))),
        GraphPathfinder.findPath(graph, coord(0, 0), coord(19, 19)));

    assertEquals(Optional.empty(), GraphPathfinder.findPath(graph, coord(19, 19), coord(0, 0)));
  }
}
