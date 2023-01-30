package frc.util.pathing;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** A map of field obstructions, used to build the field graph. */
public class FieldObstructionMap {
  // copied from 2023 json -> "length": 16.54175, "width": 8.0137
  public static final double FIELD_LENGTH = 16.54175;
  public static final double FIELD_WIDTH = 8.0137;

  public static final double FIELD_CENTER_X = FIELD_LENGTH / 2d;
  public static final double FIELD_CENTER_Y = FIELD_WIDTH / 2d;

  public enum AllianceColor {
    RED,
    BLUE,
    NEUTRAL
  }

  public static interface Obstruction {
    public boolean contains(Translation2d point);

    public String getName();

    public AllianceColor getAllianceColor();
  }

  public static class RectangleObstruction implements Obstruction {
    private final AllianceColor allianceColor;
    private final String name;
    private final Translation2d bottomLeft;
    private final Translation2d topRight;

    public RectangleObstruction(
        AllianceColor allianceColor,
        String name,
        Translation2d bottomLeft,
        Translation2d topRight) {
      this.allianceColor = allianceColor;

      this.name = name;
      this.bottomLeft = bottomLeft;
      this.topRight = topRight;
    }

    public boolean contains(Translation2d point) {
      return point.getX() >= bottomLeft.getX()
          && point.getX() <= topRight.getX()
          && point.getY() >= bottomLeft.getY()
          && point.getY() <= topRight.getY();
    }

    public String getName() {
      return name;
    }

    public AllianceColor getAllianceColor() {
      return allianceColor;
    }
  }

  private static AllianceColor invertAllianceColor(AllianceColor color) {
    switch (color) {
      case RED:
        return AllianceColor.BLUE;
      case BLUE:
        return AllianceColor.RED;
      default:
        return AllianceColor.NEUTRAL;
    }
  }

  private static void addAndMirrorRectangleObstruction(
      List<Obstruction> obstructions,
      AllianceColor allianceColor,
      String name,
      Translation2d bottomLeft,
      Translation2d topRight) {
    obstructions.add(new RectangleObstruction(allianceColor, name, bottomLeft, topRight));
    // mirror over the center line
    obstructions.add(
        new RectangleObstruction(
            invertAllianceColor(allianceColor),
            name,
            new Translation2d(FIELD_LENGTH - topRight.getX(), bottomLeft.getY()),
            new Translation2d(FIELD_LENGTH - bottomLeft.getX(), topRight.getY())));
  }

  private static List<Obstruction> initializeObstructions() {
    List<Obstruction> obstructions = new ArrayList<>();

    // dims eyeballed from field image, and cross checked with field manual and drawings

    // add the charge stations
    addAndMirrorRectangleObstruction(
        obstructions,
        AllianceColor.BLUE,
        "Charge Station",
        new Translation2d(2.919476, 1.508506),
        // + 1.933575 in x (width)
        // + 2.47015 in y (height)
        new Translation2d(4.853051, 3.978656));

    // add the scoring grid
    addAndMirrorRectangleObstruction(
        obstructions,
        AllianceColor.BLUE,
        "Scoring Grid",
        new Translation2d(0, 0),
        new Translation2d(1.37795, 5.6388));

    // add the barrier (eyeballed, and inaccurately thin to allow for scoring)
    // if your graph is less than .1m per tile, you'll need to make this thicker
    addAndMirrorRectangleObstruction(
        obstructions,
        AllianceColor.RED,
        "Barrier",
        new Translation2d(1.17, 5.43),
        new Translation2d(3.31, 5.53));

    // make the list immutable
    return Collections.unmodifiableList(obstructions);
  }

  public static final List<Obstruction> obstructions = initializeObstructions();
}
