package frc.util;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.UtilTest;
import frc.util.Graph.Edge;
import frc.util.GraphTests.Nodes;

public class GraphTests {

  enum Nodes {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
  }

  @UtilTest
  public void graphConstructs() {
    assertDoesNotThrow(() -> new Graph<Nodes>());
  }

  @UtilTest
  public void graphAddsAndStoresNodes() {
    Graph<Nodes> graph = new Graph<Nodes>();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    assertTrue(graph.getNodes().contains(Nodes.A), "Graph does not contain node A");
    assertTrue(graph.hasNode(Nodes.A), "Graph does not have node A");
    assertTrue(graph.getNodes().contains(Nodes.B), "Graph does not contain node B");
    assertTrue(graph.hasNode(Nodes.B), "Graph does not have node B");
    assertTrue(graph.getNodes().contains(Nodes.C), "Graph does not contain node C");
    assertTrue(graph.hasNode(Nodes.C), "Graph does not have node C");
  }

  @UtilTest
  public void graphReturnsReadonlySets() {
    Graph<Nodes> graph = new Graph<Nodes>();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    assertThrows(
        UnsupportedOperationException.class,
        () -> {
          graph.getNodes().add(Nodes.D);
        });

    assertThrows(
        UnsupportedOperationException.class,
        () -> {
          graph.getNeighbors(Nodes.A).add(new Edge<Nodes>(Nodes.D, 1));
        });

    assertThrows(
        UnsupportedOperationException.class,
        () -> {
          graph.getNeighbors(Nodes.A).remove(Nodes.B);
        });
  }

  @UtilTest
  public void graphAddsAndStoresEdges() {
    Graph<Nodes> graph = new Graph<Nodes>();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    graph.addEdge(Nodes.A, Nodes.B, 10);
    graph.addEdge(Nodes.A, Nodes.C, 20);
    graph.addEdge(Nodes.B, Nodes.C, 30);

    assertTrue(graph.hasEdge(Nodes.A, Nodes.B), "Graph does not contain edge A -> B");
    assertTrue(graph.hasEdge(Nodes.A, Nodes.C), "Graph does not contain edge A -> C");
    assertTrue(graph.hasEdge(Nodes.B, Nodes.C), "Graph does not contain edge B -> C");

    assertEquals(10, graph.getEdgeWeight(Nodes.A, Nodes.B), "Edge A->B has incorrect weight");
    assertEquals(20, graph.getEdgeWeight(Nodes.A, Nodes.C), "Edge A->C has incorrect weight");
    assertEquals(30, graph.getEdgeWeight(Nodes.B, Nodes.C), "Edge B->C has incorrect weight");
  }

  @UtilTest
  public void graphIsAsymmetric() {
    Graph<Nodes> graph = new Graph<Nodes>();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    graph.addEdge(Nodes.A, Nodes.B, 10);
    graph.addEdge(Nodes.A, Nodes.C, 20);
    graph.addEdge(Nodes.B, Nodes.C, 30);

    assertFalse(graph.hasEdge(Nodes.B, Nodes.A), "Graph contains edge B -> A");
    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getEdgeWeight(Nodes.B, Nodes.A),
        "getEdgeWeight B->A should be illegal");
    assertFalse(graph.hasEdge(Nodes.C, Nodes.A), "Graph contains edge C -> A");
    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getEdgeWeight(Nodes.C, Nodes.A),
        "getEdgeWeight C->A should be illegal");
    assertFalse(graph.hasEdge(Nodes.C, Nodes.B), "Graph contains edge C -> B");
    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getEdgeWeight(Nodes.C, Nodes.B),
        "getEdgeWeight C->B should be illegal");
    assertFalse(graph.hasEdge(Nodes.D, Nodes.A), "Graph contains edge D -> A");
    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getEdgeWeight(Nodes.D, Nodes.A),
        "getEdgeWeight D->A should be illegal");

    assertTrue(graph.hasEdge(Nodes.A, Nodes.B), "Graph does not contain edge A -> B");
    assertTrue(graph.hasEdge(Nodes.A, Nodes.C), "Graph does not contain edge A -> C");
    assertTrue(graph.hasEdge(Nodes.B, Nodes.C), "Graph does not contain edge B -> C");

    assertEquals(10, graph.getEdgeWeight(Nodes.A, Nodes.B), "Edge A->B has incorrect weight");
    assertEquals(20, graph.getEdgeWeight(Nodes.A, Nodes.C), "Edge A->C has incorrect weight");
    assertEquals(30, graph.getEdgeWeight(Nodes.B, Nodes.C), "Edge B->C has incorrect weight");
  }

  @UtilTest
  public void graphThrowsOnMissingNodes() {
    Graph<Nodes> graph = new Graph<Nodes>();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    graph.addEdge(Nodes.A, Nodes.B, 10);
    graph.addEdge(Nodes.A, Nodes.C, 20);
    graph.addEdge(Nodes.B, Nodes.C, 30);

    assertDoesNotThrow(
        () -> graph.hasEdge(Nodes.A, Nodes.D), "Graph throws on missing node D in hasEdge A->D");
    assertFalse(graph.hasEdge(Nodes.A, Nodes.D), "Graph contains edge A -> D");
    assertDoesNotThrow(
        () -> graph.hasEdge(Nodes.D, Nodes.A), "Graph throws on missing node D in hasEdge D->A");
    assertFalse(graph.hasEdge(Nodes.D, Nodes.A), "Graph contains edge D -> A");
    assertDoesNotThrow(
        () -> graph.hasEdge(Nodes.D, Nodes.E), "Graph throws on missing node D in hasEdge D->E");
    assertFalse(graph.hasEdge(Nodes.D, Nodes.E), "Graph contains edge D -> E");

    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getEdgeWeight(Nodes.A, Nodes.D),
        "Graph throws on missing node D in getEdgeWeight A->D");

    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getEdgeWeight(Nodes.D, Nodes.A),
        "Graph throws on missing node D in getEdgeWeight D->A");

    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getEdgeWeight(Nodes.D, Nodes.E),
        "Graph throws on missing node D in getEdgeWeight D->E");

    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getNeighbors(Nodes.D),
        "Graph throws on missing node D in getNeighbors D");

    assertThrows(
        IllegalArgumentException.class,
        () -> graph.addEdge(Nodes.A, Nodes.D, 10),
        "Graph throws on missing node D in addEdge A->D");
    assertFalse(graph.hasEdge(Nodes.A, Nodes.D), "Graph contains edge A -> D");

    assertFalse(graph.hasEdge(Nodes.A, Nodes.D), "Graph does not contain edge A -> D");
  }

  @UtilTest
  public void lockingGraphPreventsModificationAndThrowsErrors() {
    Graph<Nodes> graph = new Graph<Nodes>();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    graph.addEdge(Nodes.A, Nodes.B, 10);
    graph.addEdge(Nodes.A, Nodes.C, 20);
    graph.addEdge(Nodes.B, Nodes.C, 30);

    graph.lock();

    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addNode(Nodes.D),
        "Graph does not throw on addNode after locking");
    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addEdge(Nodes.A, Nodes.D, 10),
        "Graph does not throw on addEdge after locking");
    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addEdge(Nodes.D, Nodes.A, 10),
        "Graph does not throw on addEdge after locking");
    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addEdge(Nodes.D, Nodes.E, 10),
        "Graph does not throw on addEdge after locking");
    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addEdge(Nodes.A, Nodes.B, 10),
        "Graph does not throw on addEdge after locking");
    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addEdge(Nodes.B, Nodes.A, 10),
        "Graph does not throw on addEdge after locking");
    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addEdge(Nodes.B, Nodes.C, 10),
        "Graph does not throw on addEdge after locking");
    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addEdge(Nodes.C, Nodes.B, 10),
        "Graph does not throw on addEdge after locking");
    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addEdge(Nodes.C, Nodes.A, 10),
        "Graph does not throw on addEdge after locking");
    assertThrows(
        UnsupportedOperationException.class,
        () -> graph.addEdge(Nodes.A, Nodes.C, 10),
        "Graph does not throw on addEdge after locking");

    assertTrue(graph.hasEdge(Nodes.A, Nodes.B), "Graph does not contain edge A -> B");
    assertTrue(graph.hasEdge(Nodes.A, Nodes.C), "Graph does not contain edge A -> C");
    assertTrue(graph.hasEdge(Nodes.B, Nodes.C), "Graph does not contain edge B -> C");

    assertEquals(10, graph.getEdgeWeight(Nodes.A, Nodes.B), "Edge A->B has incorrect weight");

    assertEquals(20, graph.getEdgeWeight(Nodes.A, Nodes.C), "Edge A->C has incorrect weight");

    assertEquals(30, graph.getEdgeWeight(Nodes.B, Nodes.C), "Edge B->C has incorrect weight");
  }
}
