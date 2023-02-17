package frc.util.pathing;

import frc.util.Graph;
import frc.util.Graph.Edge;
import frc.util.MinHeap;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

/*
https://en.wikipedia.org/wiki/A*_search_algorithm
https://www.redblobgames.com/pathfinding/a-star/introduction.html
https://www.redblobgames.com/pathfinding/a-star/implementation.html

function reconstruct_path(cameFrom, current)
    total_path := {current}
    while current in cameFrom.Keys:
        current := cameFrom[current]
        total_path.prepend(current)
    return total_path

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
function A_Star(start, goal, h)
    // The set of discovered nodes that may need to be (re-)expanded.
    // Initially, only the start node is known.
    // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    openSet := {start}

    // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
    // to n currently known.
    cameFrom := an empty map

    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore := map with default value of Infinity
    gScore[start] := 0

    // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how cheap a path could be from start to finish if it goes through n.
    fScore := map with default value of Infinity
    fScore[start] := h(start)

    while openSet is not empty
        // This operation can occur in O(Log(N)) time if openSet is a min-heap or a priority queue
        current := the node in openSet having the lowest fScore[] value
        if current = goal
            return reconstruct_path(cameFrom, current)

        openSet.Remove(current)
        for each neighbor of current
            // d(current,neighbor) is the weight of the edge from current to neighbor
            // tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore := gScore[current] + d(current, neighbor)
            if tentative_gScore < gScore[neighbor]
                // This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] := current
                gScore[neighbor] := tentative_gScore
                fScore[neighbor] := tentative_gScore + h(neighbor)
                if neighbor not in openSet
                    openSet.add(neighbor)

    // Open set is empty but goal was never reached
    return failure

*/
// Remark: In this pseudocode, if a node is reached by one path, removed from openSet, and
// subsequently reached by a cheaper path, it will be added to openSet again. This is essential to
// guarantee that the path returned is optimal if the heuristic function is admissible but not
// consistent. If the heuristic is consistent, when a node is removed from openSet the path to it is
// guaranteed to be optimal so the test ‘tentative_gScore < gScore[neighbor]’ will always fail if
// the node is reached again.

/** Optimal pathfinding using A* */
public class GraphPathfinder {

  private GraphPathfinder() {}

  private static int heuristic(GridCoord a, GridCoord b, int heuristicConstant) {
    return a.getDistance(b) * heuristicConstant;
  }

  private static List<GridCoord> reconstructPath(
      HashMap<GridCoord, GridCoord> cameFrom, GridCoord current) {
    List<GridCoord> totalPath = new ArrayList<>();
    totalPath.add(current);
    while (cameFrom.containsKey(current)) {
      current = cameFrom.get(current);
      totalPath.add(0, current);
    }
    return totalPath;
  }

  /**
   * Finds the optimal path between two nodes in a graph.
   *
   * @param graph The graph to search.
   * @param start The start node.
   * @param end The end node.
   * @param heuristicConstant The heuristic constant. A higher value will make the pathfinder weight
   *     the heuristic more heavily. This sacrifices accuracy for speed, although too high a value
   *     will result in greedy depth-first search.
   * @return The optimal path, or an empty optional if no path exists.
   */
  public static Optional<List<GridCoord>> findPath(
      Graph<GridCoord> graph, GridCoord start, GridCoord end, int heuristicConstant) {

    if (!graph.hasNode(start) || !graph.hasNode(end)) {
      return Optional.empty();
    }

    /**
     * The set of discovered nodes that may need to be (re-)expanded. Initially, only the start node
     * is known.
     */
    MinHeap<GridCoord> openSet = new MinHeap<>();

    /**
     * For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
     * to n currently known.
     */
    HashMap<GridCoord, GridCoord> cameFrom = new HashMap<>();

    /** For node n, gScore[n] is the cost of the cheapest path from start to n currently known. */
    HashMap<GridCoord, Integer> gScore = new HashMap<>();
    gScore.put(start, 0);

    openSet.add(start, heuristic(start, end, heuristicConstant));

    while (!openSet.isEmpty()) {
      GridCoord current = openSet.getMin();

      if (current.equals(end)) {
        return Optional.of(reconstructPath(cameFrom, current));
      }

      for (Edge<GridCoord> neighborEdge : graph.getNeighbors(current)) {
        int tentativeGScore = gScore.get(current) + neighborEdge.weight;

        if (tentativeGScore < gScore.getOrDefault(neighborEdge.to, Integer.MAX_VALUE)) {
          cameFrom.put(neighborEdge.to, current);
          gScore.put(neighborEdge.to, tentativeGScore);
          int fScoreValue = tentativeGScore + heuristic(neighborEdge.to, end, heuristicConstant);
          // If the node is already in the open set it will be updated with the new fScore value
          openSet.add(neighborEdge.to, fScoreValue);
        }
      }
    }

    return Optional.empty();
  }
}
