package frc.util.pathing;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Pathing;
import frc.robot.Constants.Pathing.Costs;
import java.util.ArrayList;
import java.util.List;
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

  /**
   * Determine the integer distance between two grid coordinates, using {@link Costs.DIAGONAL} and
   * {@link Costs.CARDINAL} costs.
   *
   * <p>This is Chebyshev distance.
   *
   * @param other The other coordinate to measure distance to.
   * @return The distance between the two coordinates.
   */
  public int getDistance(GridCoord other) {
    int dx = Math.abs(x - other.x);
    int dy = Math.abs(y - other.y);
    return Math.min(dx, dy) * Costs.DIAGONAL + Math.abs(dx - dy) * Costs.CARDINAL;
  }

  public static List<GridCoord> line(GridCoord start, GridCoord end) {
    List<GridCoord> line = new ArrayList<>();
    int x0 = start.x;
    int y0 = start.y;
    int x1 = end.x;
    int y1 = end.y;
    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    while (true) {
      line.add(new GridCoord(x0, y0));
      if (x0 == x1 && y0 == y1) {
        break;
      }
      int e2 = 2 * err;
      if (e2 > -dy) {
        err = err - dy;
        x0 = x0 + sx;
      }
      if (e2 < dx) {
        err = err + dx;
        y0 = y0 + sy;
      }
    }
    return line;
  }

  enum LinkDirection {
    PURE_X,
    PURE_Y,
    COMBO,
    EQUAL
  }

  public static LinkDirection getLinkDirection(GridCoord start, GridCoord end) {
    if (start.x == end.x && start.y == end.y) {
      return LinkDirection.EQUAL;
    } else if (start.x == end.x && start.y != end.y) {
      return LinkDirection.PURE_Y;
    } else if (start.y == end.y && start.x != end.x) {
      return LinkDirection.PURE_X;
    } else {
      return LinkDirection.COMBO;
    }
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
