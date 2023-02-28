package frc.util.pathing;

import edu.wpi.first.math.Pair;
import frc.util.Graph;
import java.util.List;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.infra.Blackhole;

@State(Scope.Benchmark)
public class GraphPathfinderBenchmarks {
  Graph<GridCoord> graph;
  List<Pair<GridCoord, GridCoord>> paths;

  @Setup
  public void setup() {
    RubenManueverGenerator generator = new RubenManueverGenerator();
    graph = generator.adjacencyGraph;
    paths =
        List.of(
            new Pair<>(new GridCoord(149, 68), new GridCoord(23, 8)),
            new Pair<>(new GridCoord(78, 75), new GridCoord(18, 16)));
  }

  @Benchmark
  public void findOptimalPath(Blackhole bh) {
    for (var path : paths) {
      var pathResult = GraphPathfinder.findPath(graph, path.getFirst(), path.getSecond(), 1);
      bh.consume(pathResult);
    }
  }

  @Benchmark
  public void findSuboptimalPathC10(Blackhole bh) {
    for (var path : paths) {
      var pathResult = GraphPathfinder.findPath(graph, path.getFirst(), path.getSecond(), 10);
      bh.consume(pathResult);
    }
  }

  @Benchmark
  public void findSuboptimalPathC100(Blackhole bh) {
    for (var path : paths) {
      var pathResult = GraphPathfinder.findPath(graph, path.getFirst(), path.getSecond(), 100);
      bh.consume(pathResult);
    }
  }
}
