package frc.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.util.pathing.AlliancePose2d;
import java.util.List;

public class NodeSelectorUtility {
  public static record ScoreTypeIdentifier(int id) {}

  public enum NodeType {
    CONE,
    CUBE;

    // returns a unique number representing the node height combo
    public ScoreTypeIdentifier atHeight(Height height) {
      return new ScoreTypeIdentifier(this.ordinal() * 3 + height.ordinal());
    }
  }

  public enum Height {
    HIGH,
    MID,
    LOW;
  }

  public static record NodeStack(int number, AlliancePose2d position, NodeType type) {}

  private static final double X_POSITION = 1.75;

  private static final double Y_FUDGE = 0d;

  // one is on the substation, counting up to the other side
  public static final List<NodeStack> nodeStacks =
      List.of(
          new NodeStack(
              1,
              new AlliancePose2d(X_POSITION, 4.99 + Y_FUDGE, Rotation2d.fromDegrees(180)),
              NodeType.CONE),
          new NodeStack(
              2,
              new AlliancePose2d(X_POSITION, 4.43 + Y_FUDGE, Rotation2d.fromDegrees(180)),
              NodeType.CUBE),
          new NodeStack(
              3,
              new AlliancePose2d(X_POSITION, 3.87 + Y_FUDGE, Rotation2d.fromDegrees(180)),
              NodeType.CONE),
          new NodeStack(
              4,
              new AlliancePose2d(X_POSITION, 3.31 + Y_FUDGE, Rotation2d.fromDegrees(180)),
              NodeType.CONE),
          new NodeStack(
              5,
              new AlliancePose2d(X_POSITION, 2.76 + Y_FUDGE, Rotation2d.fromDegrees(180)),
              NodeType.CUBE),
          new NodeStack(
              6,
              new AlliancePose2d(X_POSITION, 2.20 + Y_FUDGE, Rotation2d.fromDegrees(180)),
              NodeType.CONE),
          new NodeStack(
              7,
              new AlliancePose2d(X_POSITION, 1.61 + Y_FUDGE, Rotation2d.fromDegrees(180)),
              NodeType.CONE),
          new NodeStack(
              8,
              new AlliancePose2d(X_POSITION, 1.08 + Y_FUDGE, Rotation2d.fromDegrees(180)),
              NodeType.CUBE),
          new NodeStack(
              9,
              new AlliancePose2d(X_POSITION, 0.25 + Y_FUDGE, Rotation2d.fromDegrees(180)),
              NodeType.CONE));

  public static final NodeStack defaultNodeStack = nodeStacks.get(0);

  public static record NodeSelection(NodeStack nodeStack, Height height) {
    @Override
    public String toString() {
      // format should be "3 high" or "5 mid" or "9 low"
      return String.format("%d %s", nodeStack.number(), height.name().toLowerCase());
    }

    public NodeSelection shift(int steps) {
      return NodeSelectorUtility.shift(this, steps);
    }

    public NodeSelection withHeight(Height height) {
      return new NodeSelection(nodeStack, height);
    }

    public ScoreTypeIdentifier getScoreTypeIdentifier() {
      return nodeStack.type().atHeight(height);
    }
  }

  public static NodeSelection shift(NodeSelection selection, int shift) {
    int index = selection.nodeStack().number() - 1;
    int newIndex = ((index + shift) % nodeStacks.size() + nodeStacks.size()) % nodeStacks.size();

    return new NodeSelection(nodeStacks.get(newIndex), selection.height());
  }
}
