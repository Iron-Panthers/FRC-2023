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

  @UtilTest
  public void nodeSelectionShiftHasCorrectSteps() {
    var selection1High =
        new NodeSelectorUtility.NodeSelection(
            NodeSelectorUtility.defaultNodeStack, NodeSelectorUtility.Height.HIGH);

    var selection2High = selection1High.shift(1);
    assertEquals("2 high", selection2High.toString());

    var selection3High = selection2High.shift(1);
    assertEquals("3 high", selection3High.toString());

    var selection5High = selection3High.shift(2);
    assertEquals("5 high", selection5High.toString());

    var selection4High = selection5High.shift(-1);
    assertEquals("4 high", selection4High.toString());
  }

  @UtilTest
  public void nodeSelectionShiftHasClamping() {
    var selection1High =
        new NodeSelectorUtility.NodeSelection(
            NodeSelectorUtility.defaultNodeStack, NodeSelectorUtility.Height.HIGH);

    var selection9High = selection1High.shift(8);
    assertEquals("9 high", selection9High.toString());

    var selection9HighAgain = selection9High.shift(1);
    assertEquals("9 high", selection9HighAgain.toString());

    var selection1HighAgain = selection9High.shift(-12);
    assertEquals("1 high", selection1HighAgain.toString());
  }
}
