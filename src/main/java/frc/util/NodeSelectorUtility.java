package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static record NodeStack(int number, Pose2d position, NodeType type) {}

  private static final double X_POSITION = 1.8;

  // one is on the substation, counting up to the other side
  public static final List<NodeStack> nodeStacks =
      List.of(
          new NodeStack(
              1, new Pose2d(X_POSITION, 4.99, Rotation2d.fromDegrees(180)), NodeType.CONE),
          new NodeStack(
              2, new Pose2d(X_POSITION, 4.43, Rotation2d.fromDegrees(180)), NodeType.CUBE),
          new NodeStack(
              3, new Pose2d(X_POSITION, 3.87, Rotation2d.fromDegrees(180)), NodeType.CONE),
          new NodeStack(
              4, new Pose2d(X_POSITION, 3.31, Rotation2d.fromDegrees(180)), NodeType.CONE),
          new NodeStack(
              5, new Pose2d(X_POSITION, 2.76, Rotation2d.fromDegrees(180)), NodeType.CUBE),
          new NodeStack(
              6, new Pose2d(X_POSITION, 2.20, Rotation2d.fromDegrees(180)), NodeType.CONE),
          new NodeStack(
              7, new Pose2d(X_POSITION, 1.61, Rotation2d.fromDegrees(180)), NodeType.CONE),
          new NodeStack(
              8, new Pose2d(X_POSITION, 1.08, Rotation2d.fromDegrees(180)), NodeType.CUBE),
          new NodeStack(
              9, new Pose2d(X_POSITION, 0.25, Rotation2d.fromDegrees(180)), NodeType.CONE));
}
