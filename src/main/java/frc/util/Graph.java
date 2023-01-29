package frc.util;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/**
 * An asymmetric, weighted graph implementation, designed for high performance for path finding
 * applications.
 *
 * <p>There is not support for removing nodes or edges.
 */
public class Graph<T> {
  private final Map<T, Map<T, Double>> internalBiHashMap = new HashMap<>();

  /**
   * Returns the immutable version of a set, to prevent handing out the ability to modify the
   * internal data structure.
   *
   * @param <T> The type of the set.
   * @param set The set to make immutable.
   * @return The immutable version of the set.
   */
  private static <T> Set<T> view(Set<T> set) {
    return Collections.unmodifiableSet(set);
  }

  private Map<T, Double> safeGet(T node) {
    return internalBiHashMap.computeIfAbsent(node, k -> new HashMap<>());
  }

  /**
   * Adds a node to the graph.
   *
   * @param node the node to add
   */
  public void addNode(T node) {
    internalBiHashMap.put(node, new HashMap<>());
  }

  /**
   * Adds an edge to the graph.
   *
   * @param from the node the edge is from
   * @param to the node the edge is to
   * @param weight the weight of the edge
   */
  public void addEdge(T from, T to, double weight) {
    safeGet(from).put(to, weight);
  }

  /**
   * Gets the weight of an edge without the overhead of creating an Optional. This method only
   * belongs in hot code loops. You probably don't care about the overhead of an Optional, and
   * should use {@link #getEdgeWeight(T, T)} . Be careful.
   *
   * @param from the node the edge is from
   * @param to the node the edge is to
   * @return the weight of the edge, or null if the edge does not exist
   */
  public double getNullableEdgeWeight(T from, T to) {
    return safeGet(from).get(to);
  }

  /**
   * Gets the weight of an edge.
   *
   * @param from the node the edge is from
   * @param to the node the edge is to
   * @return the weight of the edge if it exists, otherwise empty
   */
  public Optional<Double> getEdgeWeight(T from, T to) {
    return Optional.ofNullable(safeGet(from).get(to));
  }

  /**
   * Gets the neighbors of a node.
   *
   * @param node the node to get the neighbors of
   * @return the neighbors of the node
   */
  public Set<T> getNeighbors(T node) {
    return view(safeGet(node).keySet());
  }

  /**
   * Gets the nodes in the graph.
   *
   * @return the nodes in the graph
   */
  public Set<T> getNodes() {
    return view(internalBiHashMap.keySet());
  }

  public boolean hasNode(T node) {
    return internalBiHashMap.containsKey(node);
  }

  public boolean hasEdge(T from, T to) {
    return safeGet(from).containsKey(to);
  }

  public String[] visualize() {
    String[] lines = new String[internalBiHashMap.size() + 1];
    lines[0] = "digraph G {";
    int i = 1;
    for (T node : internalBiHashMap.keySet()) {
      for (T neighbor : internalBiHashMap.get(node).keySet()) {
        lines[i++] =
            "  "
                + node
                + " -> "
                + neighbor
                + " [label=\""
                + internalBiHashMap.get(node).get(neighbor)
                + "\"];";
      }
    }
    lines[lines.length - 1] = "}";
    return lines;
  }
}
