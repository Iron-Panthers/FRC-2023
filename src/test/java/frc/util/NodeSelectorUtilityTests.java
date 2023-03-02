package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.UtilTest;

public class NodeSelectorUtilityTests {

  @UtilTest
  public void nodeSelectionToStringIsCorrect() {

    var selection1High =
        new NodeSelectorUtility.NodeSelection(
            NodeSelectorUtility.defaultNodeStack, NodeSelectorUtility.Height.HIGH);

    assertEquals("1 high", selection1High.toString());

    var selection3Mid =
        new NodeSelectorUtility.NodeSelection(
            NodeSelectorUtility.nodeStacks.get(2), NodeSelectorUtility.Height.MID);
    assertEquals("3 mid", selection3Mid.toString());

    var selection9Low =
        new NodeSelectorUtility.NodeSelection(
            NodeSelectorUtility.nodeStacks.get(8), NodeSelectorUtility.Height.LOW);

    assertEquals("9 low", selection9Low.toString());
  }
}
