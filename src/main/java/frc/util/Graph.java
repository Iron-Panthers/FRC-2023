package frc.util;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * An asymmetric, weighted graph implementation, designed for high performance for path finding
 * applications.
 *
 * <p>There is not support for removing nodes or edges.
 */
public class Graph<T> {
  private Map<T, Map<T, Double>> internalBiHashMap = new HashMap<>();

  private boolean locked = false;

  /** Creates a new graph. */
  public Graph() {}

  /** lock the graph such that no more mutations can occur. */
  public void lock() {
    internalBiHashMap.forEach(
        (key, value) -> internalBiHashMap.put(key, Collections.unmodifiableMap(value)));
    internalBiHashMap = Collections.unmodifiableMap(internalBiHashMap);
    locked = true;
  }

  private void checkLocked() {
    if (locked) {
      throw new UnsupportedOperationException("Graph is locked");
    }
  }

  private void assertNodeExists(T node) {
    if (!internalBiHashMap.containsKey(node)) {
      throw new IllegalArgumentException("Node does not exist " + node.toString());
    }
  }

  private void assertNodeDoesNotExist(T node) {
    if (internalBiHashMap.containsKey(node)) {
      throw new IllegalArgumentException("Node already exists " + node.toString());
    }
  }

  private void assertEdgeExists(T from, T to) {
    assertNodeExists(to);
    assertNodeExists(from);
    if (!internalBiHashMap.get(from).containsKey(to)) {
      throw new IllegalArgumentException(
          "Edge does not exist " + from.toString() + " -> " + to.toString());
    }
  }

  private void assertEdgeDoesNotExist(T from, T to) {
    if (internalBiHashMap.get(from).containsKey(to)) {
      throw new IllegalArgumentException(
          "Edge already exists " + from.toString() + " -> " + to.toString());
    }
  }

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

  /**
   * Adds a node to the graph if it does not already exist.
   *
   * @param node the node to add
   */
  public void addNode(T node) {
    checkLocked();
    assertNodeDoesNotExist(node);
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
    checkLocked();
    assertEdgeDoesNotExist(from, to);
    internalBiHashMap.get(from).put(to, weight);
  }

  /**
   * Gets the weight of an edge.
   *
   * @param from the node the edge is from
   * @param to the node the edge is to
   * @return the weight of the edge
   */
  public double getEdgeWeight(T from, T to) {
    assertEdgeExists(from, to);
    return internalBiHashMap.get(from).get(to);
  }

  /**
   * Gets the neighbors of a node.
   *
   * @param node the node to get the neighbors of
   * @return the neighbors of the node, or an empty set if the node does not exist
   */
  public Set<T> getNeighbors(T node) {
    assertNodeExists(node);
    return view(internalBiHashMap.get(node).keySet());
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
    return hasNode(from) && internalBiHashMap.get(from).containsKey(to);
  }
}
