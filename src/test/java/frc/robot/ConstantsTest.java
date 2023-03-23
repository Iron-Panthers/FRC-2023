package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.UtilTest;
import frc.util.NodeSelectorUtility;

public class ConstantsTest {
  @UtilTest
  public void scoreStepMapHasEveryType() {
    for (var type : NodeSelectorUtility.NodeType.values()) {
      for (var height : NodeSelectorUtility.Height.values()) {
        var id = type.atHeight(height);
        assertTrue(
            Constants.SCORE_STEP_MAP.containsKey(id),
            String.format("Missing %s %s %s in SCORE_STEP_MAP", type, height, id));
      }
    }
  }
}
