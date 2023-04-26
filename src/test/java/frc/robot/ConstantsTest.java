package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.RobotTest;
import java.lang.reflect.Field;

public class ConstantsTest {

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
