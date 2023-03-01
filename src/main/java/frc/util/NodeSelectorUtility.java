package frc.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class NodeSelectorUtility {
  public enum NodeType {
    CONE,
    CUBE,
    ANY,
  }

  public enum Height {
    HYBRID,
    MIDDLE,
    HIGH,
  }

  public static final int WIDTH = 9;
  public static final int HEIGHT = 3;

  public static record NodeStack(int number, Translation2d position, NodeType type) {}

  // one is on the substation, counting up to the other side
  public static List<NodeStack> nodeStacks =
      List.of(
          new NodeStack(1, null, NodeType.CONE),
          new NodeStack(2, null, NodeType.CUBE),
          new NodeStack(3, null, NodeType.CONE),
          new NodeStack(4, null, NodeType.CONE),
          new NodeStack(5, null, NodeType.CUBE),
          new NodeStack(6, null, NodeType.CONE),
          new NodeStack(7, null, NodeType.CONE),
          new NodeStack(8, null, NodeType.CUBE),
          new NodeStack(9, null, NodeType.CONE));
}
