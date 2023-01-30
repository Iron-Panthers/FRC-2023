package frc.util;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.UtilTest;
import java.util.Optional;

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
    assertTrue(graph.getNodes().contains(Nodes.B), "Graph does not contain node B");
    assertTrue(graph.getNodes().contains(Nodes.C), "Graph does not contain node C");
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
          graph.getNeighbors(Nodes.A).get().add(Nodes.D);
        });

    assertThrows(
        UnsupportedOperationException.class,
        () -> {
          graph.getNeighbors(Nodes.A).get().remove(Nodes.B);
        });
  }

  @UtilTest
  public void graphAddsAndStoresEdges() {
    Graph<Nodes> graph = new Graph<Nodes>();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    graph.addEdge(Nodes.A, Nodes.B, 1.0);
    graph.addEdge(Nodes.A, Nodes.C, 2.0);
    graph.addEdge(Nodes.B, Nodes.C, 3.0);

    assertTrue(graph.hasEdge(Nodes.A, Nodes.B), "Graph does not contain edge A -> B");
    assertTrue(graph.hasEdge(Nodes.A, Nodes.C), "Graph does not contain edge A -> C");
    assertTrue(graph.hasEdge(Nodes.B, Nodes.C), "Graph does not contain edge B -> C");

    assertEquals(
        1.0, graph.getNullableEdgeWeight(Nodes.A, Nodes.B), "Edge A->B has incorrect weight");
    assertEquals(
        2.0, graph.getNullableEdgeWeight(Nodes.A, Nodes.C), "Edge A->C has incorrect weight");
    assertEquals(
        3.0, graph.getNullableEdgeWeight(Nodes.B, Nodes.C), "Edge B->C has incorrect weight");
  }

  @UtilTest
  public void graphIsAsymmetric() {
    Graph<Nodes> graph = new Graph<Nodes>();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    graph.addEdge(Nodes.A, Nodes.B, 1.0);
    graph.addEdge(Nodes.A, Nodes.C, 2.0);
    graph.addEdge(Nodes.B, Nodes.C, 3.0);

    assertFalse(graph.hasEdge(Nodes.B, Nodes.A), "Graph contains edge B -> A");
    assertFalse(graph.hasEdge(Nodes.C, Nodes.A), "Graph contains edge C -> A");
    assertFalse(graph.hasEdge(Nodes.C, Nodes.B), "Graph contains edge C -> B");

    assertTrue(graph.hasEdge(Nodes.A, Nodes.B), "Graph does not contain edge A -> B");
    assertTrue(graph.hasEdge(Nodes.A, Nodes.C), "Graph does not contain edge A -> C");
    assertTrue(graph.hasEdge(Nodes.B, Nodes.C), "Graph does not contain edge B -> C");

    assertEquals(
        1.0, graph.getNullableEdgeWeight(Nodes.A, Nodes.B), "Edge A->B has incorrect weight");
    assertEquals(
        2.0, graph.getNullableEdgeWeight(Nodes.A, Nodes.C), "Edge A->C has incorrect weight");
    assertEquals(
        3.0, graph.getNullableEdgeWeight(Nodes.B, Nodes.C), "Edge B->C has incorrect weight");
  }

  @UtilTest
  public void graphDoesNotThrowOnMissingNodes() {
    Graph<Nodes> graph = new Graph<Nodes>();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    graph.addEdge(Nodes.A, Nodes.B, 1.0);
    graph.addEdge(Nodes.A, Nodes.C, 2.0);
    graph.addEdge(Nodes.B, Nodes.C, 3.0);

    assertDoesNotThrow(
        () -> graph.hasEdge(Nodes.A, Nodes.D), "Graph throws on missing node D in hasEdge A->D");
    assertDoesNotThrow(
        () -> graph.hasEdge(Nodes.D, Nodes.A), "Graph throws on missing node D in hasEdge D->A");
    assertDoesNotThrow(
        () -> graph.hasEdge(Nodes.D, Nodes.E), "Graph throws on missing node D in hasEdge D->E");

    assertDoesNotThrow(
        () -> graph.getEdgeWeight(Nodes.A, Nodes.D),
        "Graph throws on missing node D in getEdgeWeight A->D");
    assertDoesNotThrow(
        () -> graph.getEdgeWeight(Nodes.D, Nodes.A),
        "Graph throws on missing node D in getEdgeWeight D->A");
    assertDoesNotThrow(
        () -> graph.getEdgeWeight(Nodes.D, Nodes.E),
        "Graph throws on missing node D in getEdgeWeight D->E");

    assertDoesNotThrow(
        () -> graph.getNeighbors(Nodes.D), "Graph throws on missing node D in getNeighbors D");

    assertDoesNotThrow(
        () -> graph.addEdge(Nodes.A, Nodes.D, 1.0),
        "Graph throws on missing node D in addEdge A->D");

    assertEquals(
        Optional.of(1.0),
        graph.getEdgeWeight(Nodes.A, Nodes.D),
        "Edge A->D has incorrect weight or wasn't implicitly added");
  }

  @UtilTest
  public void graphThrowsOnImplicitWhenStrict() {
    Graph<Nodes> graph = Graph.strict();
    graph.addNode(Nodes.A);
    graph.addNode(Nodes.B);
    graph.addNode(Nodes.C);

    graph.addEdge(Nodes.A, Nodes.B, 1.0);
    graph.addEdge(Nodes.A, Nodes.C, 2.0);
    graph.addEdge(Nodes.B, Nodes.C, 3.0);

    assertDoesNotThrow(
        () -> graph.hasEdge(Nodes.A, Nodes.D),
        "Strict graph throws on missing node but not implicitly constructed D in hasEdge A->D");
    assertThrows(
        IllegalArgumentException.class,
        () -> graph.hasEdge(Nodes.D, Nodes.A),
        "Strict graph does not throw on missing node D in hasEdge D->A");
    assertThrows(
        IllegalArgumentException.class,
        () -> graph.hasEdge(Nodes.D, Nodes.E),
        "Strict graph does not throw on missing node D in hasEdge D->E");

    assertDoesNotThrow(
        () -> graph.getEdgeWeight(Nodes.A, Nodes.D),
        "Strict graph throws on missing node but not implicitly constructed D in getEdgeWeight A->D");

    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getEdgeWeight(Nodes.D, Nodes.A),
        "Strict graph does not throw on missing node D in getEdgeWeight D->A");

    assertThrows(
        IllegalArgumentException.class,
        () -> graph.getEdgeWeight(Nodes.D, Nodes.E),
        "Strict graph does not throw on missing node D in getEdgeWeight D->E");

    assertDoesNotThrow(
        () -> graph.getNeighbors(Nodes.D),
        "Strict graph throws on missing node D in getNeighbors D");

    assertEquals(
        Optional.empty(),
        graph.getNeighbors(Nodes.D),
        "Strict graph does not return empty neighbors for missing node D");
  }
}
