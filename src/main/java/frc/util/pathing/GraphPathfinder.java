package frc.util.pathing;

import frc.util.Graph;
import java.util.List;
import java.util.Optional;

/** Optimal pathfinding using dijkstra's algorithm with contraction hierarchies. */
public class GraphPathfinder {

  /**
   * Finds the optimal path between two nodes in a graph.
   *
   * @param <T> The type of the node keys.
   * @param graph The graph to search.
   * @param start The start node.
   * @param end The end node.
   * @return The optimal path, or an empty optional if no path exists.
   */
  public static <T> Optional<List<T>> findPath(Graph<T> graph, T start, T end) {
    // TODO: Implement
    return Optional.empty();
  }
}
