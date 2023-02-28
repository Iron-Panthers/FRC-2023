package frc.util.pathing;

/**
 * A grid of boolean values, with a fixed width and height. 1d array internally. No bounds checks.
 * Not generic to save a bit of memory with boolean instead of auto boxing to Boolean.
 */
public class BoolGrid {
  private final int width;
  private final int height;
  private final boolean[] grid;

  /**
   * Create a new grid.
   *
   * @param width The width of the grid.
   * @param height The height of the grid.
   */
  public BoolGrid(int width, int height) {
    this.width = width;
    this.height = height;
    this.grid = new boolean[width * height];
  }

  /**
   * Get the value at a given coordinate.
   *
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @return The value at the given coordinate.
   */
  public boolean get(int x, int y) {
    return grid[y * width + x];
  }

  /**
   * Get the value at a given coordinate.
   *
   * @param coord The coordinate.
   */
  public boolean get(GridCoord coord) {
    return grid[coord.y * width + coord.x];
  }

  /**
   * Set the value at a given coordinate.
   *
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @param value The value to set.
   */
  public void set(int x, int y, boolean value) {
    grid[y * width + x] = value;
  }

  /**
   * Set the value at a given coordinate.
   *
   * @param coord The coordinate.
   * @param value The value to set.
   */
  public void set(GridCoord coord, boolean value) {
    grid[coord.y * width + coord.x] = value;
  }

  /**
   * Get the width of the grid.
   *
   * @return The width of the grid.
   */
  public int getWidth() {
    return width;
  }

  /**
   * Get the height of the grid.
   *
   * @return The height of the grid.
   */
  public int getHeight() {
    return height;
  }
}
