package frc.util;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;

/**
 * An asymmetric, weighted graph implementation, designed for high performance for path finding
 * applications.
 *
 * <p>There is not support for removing nodes or edges.
 */
public class Graph<T> {
  private final Map<T, Map<T, Double>> internalBiHashMap = new HashMap<>();

  private Consumer<T> implicitNodeKeyAddedCallback = k -> {};

  /**
   * Creates a new graph, with a function to be called when a node is added implicitly.
   *
   * @param implicitNodeKeyAddedCallback The function to be called when a node is added implicitly,
   *     by being missing and the first param in an accessor or setter.
   */
  public Graph(Consumer<T> implicitNodeKeyAddedCallback) {
    this.implicitNodeKeyAddedCallback = implicitNodeKeyAddedCallback;
  }

  /** Creates a new graph, that allows implicit node creation. */
  public Graph() {}

  /**
   * Creates a new graph, that allows implicit node creation, but prints to System.out.err when it
   * happens.
   *
   * @param <T> The type of the node keys.
   * @return A new graph, that allows implicit node creation, but prints to System.out.err when it
   *     happens.
   */
  public static <T> Graph<T> warnOnImplicitNodeKeyAdded() {
    return new Graph<>(k -> System.err.println("Implicitly added node key: " + k));
  }

  /**
   * Creates a new graph, that throws an exception when a node is added implicitly--although it will
   * still add it. This is useful for debugging and validating that your graph is behaving as you
   * expect, although is likely unsuited for production robot code.
   *
   * @param <T> The type of the node keys.
   * @return A new graph, that throws an IllegalArgumentException exception when a node is added
   *     implicitly.
   */
  public static <T> Graph<T> strict() {
    return new Graph<>(
        k -> {
          throw new IllegalArgumentException("Implicitly added node key: " + k);
        });
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

  private Map<T, Double> safeGet(T node) {
    return internalBiHashMap.computeIfAbsent(
        node,
        k -> {
          implicitNodeKeyAddedCallback.accept(k);
          return new HashMap<>();
        });
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
}
