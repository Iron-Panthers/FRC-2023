package frc.util;

import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.State;

@State(Scope.Benchmark)
public class GraphBenchmark {
  private static String[] nodes = {
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S",
    "T", "U", "V", "W", "X", "Y", "Z"
  };

  @Benchmark
  public Graph<String> constructGraph() {
    Graph<String> graph = new Graph<>();
    for (String node : nodes) {
      graph.addNode(node);
    }
    for (int i = 0; i < nodes.length; i++) {
      for (int j = i + 1; j < nodes.length; j++) {
        graph.addEdge(nodes[i], nodes[j], i * j);
      }
    }
    graph.lock();
    return graph;
  }
}
