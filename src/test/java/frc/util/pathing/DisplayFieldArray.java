package frc.util.pathing;

public class DisplayFieldArray {
  public static enum FieldSquare {
    OBSTRUCTION("##"),
    EMPTY("  "),
    DANGER("//"),
    PATH("::"),
    START("SS"),
    END("EE"),
    REDUNDANT_CRITICAL_POINT("XX"),
    CRITICAL_POINT("<>"),
    SPLINE("~~"),
    X_AXIS_FLOW(">."),
    Y_AXIS_FLOW("v.");

    private final String display;

    FieldSquare(String display) {
      this.display = display;
    }
  }

  public static void renderField(StringBuilder sb, FieldSquare[][] field) {
    final int xMax = field.length;
    final int yMax = field[0].length;
    for (int y = yMax - 1; y >= 0; y--) {
      for (int x = 0; x < xMax; x++) {
        sb.append(field[x][y].display);
      }
      sb.append("\n");
    }
  }
}
