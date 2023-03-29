package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.RobotTest;
import frc.util.NodeSelectorUtility;
import java.lang.reflect.Field;

public class ConstantsTest {
  @RobotTest
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

  private void recursiveFinalCheck(Class<?> clazz) {
    // iterate internal classes
    for (Class<?> internalClazz : clazz.getDeclaredClasses()) {
      recursiveFinalCheck(internalClazz);
    }
    for (Field field : clazz.getDeclaredFields()) {
      if (field.getType().isPrimitive() || field.getType().isArray() || field.getType().isEnum()) {
        assertTrue(
            java.lang.reflect.Modifier.isFinal(field.getModifiers()),
            String.format("Field %s in class %s is not final", field.getName(), clazz.getName()));
      }
    }
  }

  @RobotTest
  public void everyNestedMemberIsFinal() {
    recursiveFinalCheck(Constants.class);
  }
}
