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
  private Map<T, Map<T, Double>> internalBiHashMap = new HashMap<>();

  /** Creates a new graph. */
  public Graph() {}

  /** lock the graph such that no more mutations can occur. */
  public void lock() {
    internalBiHashMap.forEach(
        (key, value) -> internalBiHashMap.put(key, Collections.unmodifiableMap(value)));
    internalBiHashMap = Collections.unmodifiableMap(internalBiHashMap);
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

  private Map<T, Double> getOrConstruct(T node) {
    return internalBiHashMap.computeIfAbsent(node, k -> new HashMap<>());
  }

  /**
   * Adds a node to the graph if it does not already exist. If the node already exists, this method
   * does nothing.
   *
   * @param node the node to add
   */
  public void addNode(T node) {
    if (!hasNode(node)) {
      internalBiHashMap.put(node, new HashMap<>());
    }
  }

  /**
   * Adds an edge to the graph.
   *
   * @param from the node the edge is from
   * @param to the node the edge is to
   * @param weight the weight of the edge
   */
  public void addEdge(T from, T to, double weight) {
    getOrConstruct(from).put(to, weight);
    // if the "to" node doesn't exist, it will be implicitly created here
    getOrConstruct(to);
  }

  /**
   * Gets the weight of an edge without the overhead of creating an Optional. This method only
   * belongs in hot code loops. You probably don't care about the overhead of an Optional, and
   * should use {@link #getEdgeWeight(T, T)}. Be careful.
   *
   * @param from the node the edge is from
   * @param to the node the edge is to
   * @return the weight of the edge, or null if the edge does not exist
   */
  public double getNullableEdgeWeight(T from, T to) {
    return hasNode(from) ? internalBiHashMap.get(from).get(to) : null;
  }

  /**
   * Gets the weight of an edge.
   *
   * @param from the node the edge is from
   * @param to the node the edge is to
   * @return the weight of the edge if it exists, otherwise empty
   */
  public Optional<Double> getEdgeWeight(T from, T to) {
    return Optional.ofNullable(getNullableEdgeWeight(from, to));
  }

  /**
   * Gets the neighbors of a node without the overhead of creating an Optional. This method only
   * belongs in hot code loops. You probably don't care about the overhead of an Optional, and
   * should use {@link #getNeighbors(T, T)}. Be careful.
   *
   * @param node the node to get the neighbors of
   * @return the neighbors of the node, or an null if the node does not exist
   */
  public Set<T> getNullableNeighbors(T node) {
    return hasNode(node) ? view(internalBiHashMap.get(node).keySet()) : null;
  }

  /**
   * Gets the neighbors of a node.
   *
   * @param node the node to get the neighbors of
   * @return the neighbors of the node, or an empty set if the node does not exist
   */
  public Optional<Set<T>> getNeighbors(T node) {
    return Optional.ofNullable(getNullableNeighbors(node));
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
