package frc;

import static java.lang.Character.isLowerCase;
import static java.lang.Character.isUpperCase;
import static java.lang.Character.toLowerCase;

import java.lang.reflect.Method;
import org.junit.jupiter.api.DisplayNameGenerator;

public class ReplaceCamelCase extends DisplayNameGenerator.Standard {
  @Override
  public String generateDisplayNameForClass(Class<?> testClass) {
    return replaceCamelCase(super.generateDisplayNameForClass(testClass));
  }

  @Override
  public String generateDisplayNameForNestedClass(Class<?> nestedClass) {
    return replaceCamelCase(super.generateDisplayNameForNestedClass(nestedClass));
  }

  @Override
  public String generateDisplayNameForMethod(Class<?> testClass, Method testMethod) {
    return this.replaceCamelCase(testMethod.getName())
        + DisplayNameGenerator.parameterTypesAsString(testMethod);
  }

  String replaceCamelCase(String camelCase) {
    StringBuilder result = new StringBuilder();
    char[] charArray = camelCase.toCharArray();
    for (int i = 0; i < charArray.length; i++) {
      char ch = charArray[i];

      if (isLowerCase(ch)) {
        result.append(ch);
        continue;
      }

      // if the first letter is uppercase, we don't want a space before it
      if (isUpperCase(ch) && i == 0) {
        result.append(toLowerCase(ch));
        continue;
      }

      char prevCh = i == 0 ? ' ' : charArray[i - 1];
      char nextCh = i == charArray.length - 1 ? ' ' : charArray[i + 1];

      // ' ' is not lower or upper case, so we use !isUpperCase to treat it as lower case. ' ' will
      // never appear in input strings

      if (isUpperCase(ch) && !isUpperCase(nextCh) && !isUpperCase(prevCh)) {
        result.append(' ');
        result.append(toLowerCase(ch));
        continue;
      }

      // in this scenario, the current ch is uppercase along with the next one, but the prev one is
      // not. This means we have a chunk of capitals
      if (!isUpperCase(prevCh)) {
        result.append(' ');
        result.append(ch);
        continue;
      }

      // in this scenario, we are at the end of a set of capitals, but not the end of the string, so
      // the prev capital is actually a
      // new space
      if (i < charArray.length - 2 && isUpperCase(nextCh) && isLowerCase(charArray[i + 2])) {
        result.append(ch);
        result.append(' ');
        result.append(toLowerCase(nextCh));
        result.append(charArray[i + 2]);
        i += 2; // we consumed three chars
        continue;
      }

      result.append(ch);
    }
    if (isLowerCase(charArray[0])) result.append(' ');
    return result.toString();
  }
}
