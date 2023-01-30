package frc.util.pathing;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Pathing;
import java.util.Objects;

public class GridCoord {
  public final int x;
  public final int y;

  public GridCoord(int x, int y) {
    this.x = x;
    this.y = y;
  }

  public GridCoord(Translation2d coord) {
    this.x = (int) Math.round(coord.getX() / Pathing.CELL_SIZE_METERS);
    this.y = (int) Math.round(coord.getY() / Pathing.CELL_SIZE_METERS);
  }

  public Translation2d toTranslation2d() {
    return new Translation2d(x * Pathing.CELL_SIZE_METERS, y * Pathing.CELL_SIZE_METERS);
  }

  public double getDistance(GridCoord other) {
    return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2));
  }

  @Override
  public boolean equals(Object o) {
    if (o instanceof GridCoord) {
      return ((GridCoord) o).x == x && ((GridCoord) o).y == y;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(x, y);
  }

  @Override
  public String toString() {
    return "GridCoord{ " + x + " " + y + " }";
  }
}
