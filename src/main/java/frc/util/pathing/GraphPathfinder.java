package frc.util.pathing;

import edu.wpi.first.math.geometry.Translation2d;
import frc.util.Graph;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;

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

  private static double heuristic(Translation2d a, Translation2d b) {
    return a.getDistance(b);
  }

  private static List<Translation2d> reconstructPath(
      HashMap<Translation2d, Translation2d> cameFrom, Translation2d current) {
    List<Translation2d> totalPath = new ArrayList<>();
    totalPath.add(current);
    while (cameFrom.containsKey(current)) {
      current = cameFrom.get(current);
      totalPath.add(0, current);
    }
    return totalPath;
  }

  private static class Node implements Comparable<Node> {
    private final Translation2d translation2d;
    private final double fScore;

    public Node(Translation2d translation2d, double fScore) {
      this.translation2d = translation2d;
      this.fScore = fScore;
    }

    @Override
    public int compareTo(Node other) {
      return Double.compare(fScore, other.fScore);
    }

    @Override
    public boolean equals(Object other) {
      if (other == null) return false;
      if (other == this) return true;
      if (!(other instanceof Node)) return false;

      Node otherNode = (Node) other;
      return translation2d.equals(otherNode.translation2d);
    }

    @Override
    public int hashCode() {
      // this is evil but fine here
      return translation2d.hashCode();
    }
  }

  /**
   * Finds the optimal path between two nodes in a graph.
   *
   * @param graph The graph to search.
   * @param start The start node.
   * @param end The end node.
   * @return The optimal path, or an empty optional if no path exists.
   */
  public static Optional<List<Translation2d>> findPath(
      Graph<Translation2d> graph, Translation2d start, Translation2d end) {
    /**
     * The set of discovered nodes that may need to be (re-)expanded. Initially, only the start node
     * is known.
     */
    PriorityQueue<Node> openSet = new PriorityQueue<>(Collections.reverseOrder());

    /**
     * For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
     * to n currently known.
     */
    HashMap<Translation2d, Translation2d> cameFrom = new HashMap<>();

    /** For node n, gScore[n] is the cost of the cheapest path from start to n currently known. */
    HashMap<Translation2d, Double> gScore = new HashMap<>();
    gScore.put(start, 0d);

    /**
     * For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
     * how cheap a path could be from start to finish if it goes through n.
     */
    HashMap<Translation2d, Double> fScore = new HashMap<>();
    fScore.put(start, heuristic(start, end));

    openSet.add(new Node(start, fScore.get(start)));

    while (!openSet.isEmpty()) {
      Node currentNode = openSet.poll();
      Translation2d current = currentNode.translation2d;

      if (current.equals(end)) {
        return Optional.of(reconstructPath(cameFrom, current));
      }

      for (Translation2d neighbor : graph.getNeighbors(current)) {
        double tentativeGScore = gScore.get(current) + current.getDistance(neighbor);

        if (tentativeGScore < gScore.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
          cameFrom.put(neighbor, current);
          gScore.put(neighbor, tentativeGScore);
          fScore.put(neighbor, tentativeGScore + heuristic(neighbor, end));

          Node neighborNode = new Node(neighbor, fScore.get(neighbor));

          if (!openSet.contains(neighborNode)) {
            openSet.add(neighborNode);
          }
        }
      }
    }

    return Optional.empty();
  }
}
