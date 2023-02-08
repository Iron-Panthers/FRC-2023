package frc.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * An asymmetric, weighted graph implementation, designed for high performance for path finding
 * applications.
 *
 * <p>There is not support for removing nodes or edges.
 */
public class Graph<T> {
  /** An edge in the graph. Stores the destination node and the weight of the edge. */
  public static class Edge<T> {
    public final T to;
    public final int weight;

    public Edge(T to, int weight) {
      this.to = to;
      this.weight = weight;
    }

    @Override
    public int hashCode() {
      return to.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
      if (obj == null) return false;
      if (!(obj instanceof Edge)) return false;
      Edge<?> other = (Edge<?>) obj;
      return to.equals(other.to);
    }
  }

  private Map<T, List<Edge<T>>> internalHashMap = new HashMap<>();

  private boolean locked = false;

  /** Creates a new graph. */
  public Graph() {}

  /** lock the graph such that no more mutations can occur. */
  public void lock() {
    internalHashMap.forEach(
        (key, value) -> internalHashMap.put(key, Collections.unmodifiableList(value)));
    internalHashMap = Collections.unmodifiableMap(internalHashMap);
    locked = true;
  }

  private void checkLocked() {
    if (locked) {
      throw new UnsupportedOperationException("Graph is locked");
    }
  }

  private void assertNodeExists(T node) {
    if (!internalHashMap.containsKey(node)) {
      throw new IllegalArgumentException("Node does not exist " + node.toString());
    }
  }

  private void assertNodeDoesNotExist(T node) {
    if (internalHashMap.containsKey(node)) {
      throw new IllegalArgumentException("Node already exists " + node.toString());
    }
  }

  private Edge<T> internalGetEdge(T from, T to) {
    assertNodeExists(from);
    assertNodeExists(to);
    for (Edge<T> edge : internalHashMap.get(from)) {
      if (edge.to.equals(to)) {
        return edge;
      }
    }
    return null;
  }

  private void assertEdgeExists(T from, T to) {
    if (internalGetEdge(from, to) == null) {
      throw new IllegalArgumentException(
          "Edge does not exist " + from.toString() + " -> " + to.toString());
    }
  }

  private void assertEdgeDoesNotExist(T from, T to) {
    if (internalGetEdge(from, to) != null) {
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

  private static <T> List<T> view(List<T> list) {
    return Collections.unmodifiableList(list);
  }

  /**
   * Adds a node to the graph if it does not already exist.
   *
   * @param node the node to add
   */
  public void addNode(T node) {
    checkLocked();
    assertNodeDoesNotExist(node);
    internalHashMap.put(node, new ArrayList<>());
  }

  /**
   * Adds an edge to the graph.
   *
   * @param from the node the edge is from
   * @param to the node the edge is to
   * @param weight the weight of the edge
   */
  public void addEdge(T from, T to, int weight) {
    checkLocked();
    assertEdgeDoesNotExist(from, to);
    internalHashMap.get(from).add(new Edge<>(to, weight));
  }

  /**
   * Gets the weight of an edge.
   *
   * @param from the node the edge is from
   * @param to the node the edge is to
   * @return the weight of the edge
   */
  public int getEdgeWeight(T from, T to) {
    assertNodeExists(from);
    assertNodeExists(to);
    for (Edge<T> edge : internalHashMap.get(from)) {
      if (edge.to.equals(to)) {
        return edge.weight;
      }
    }
    throw new IllegalArgumentException(
        "Edge does not exist " + from.toString() + " -> " + to.toString());
  }

  /**
   * Gets the neighbors of a node.
   *
   * @param node the node to get the neighbors of
   * @return the neighbors of the node, as a list of edges
   */
  public List<Edge<T>> getNeighbors(T node) {
    assertNodeExists(node);
    return view(internalHashMap.get(node));
  }

  /**
   * Gets the nodes in the graph.
   *
   * @return the nodes in the graph
   */
  public Set<T> getNodes() {
    return view(internalHashMap.keySet());
  }

  public boolean hasNode(T node) {
    return internalHashMap.containsKey(node);
  }

  public boolean hasEdge(T from, T to) {
    if (!hasNode(from) || !hasNode(to)) return false;

    for (Edge<T> edge : internalHashMap.get(from)) {
      if (edge.to.equals(to)) {
        return true;
      }
    }

    return false;
  }
}
