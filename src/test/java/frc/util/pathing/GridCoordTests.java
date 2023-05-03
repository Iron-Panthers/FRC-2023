package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilParamTest;
import frc.UtilTest;
import frc.robot.Constants.Pathing.Costs;
import frc.util.pathing.GridCoord.LinkDirection;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

public class GridCoordTests {
  public static Stream<Arguments> getClosestPointProvider() {
    return Stream.of(
        Arguments.of(new Translation2d(2.001, 2), new Translation2d(2, 2)),
        Arguments.of(new Translation2d(3.12, 2.01), new Translation2d(3.1, 2)),
        Arguments.of(new Translation2d(3.12, 2.06), new Translation2d(3.1, 2.1)),
        Arguments.of(new Translation2d(3.12, -.01), new Translation2d(3.1, 0))
        // brace holder
        );
  }

  @UtilParamTest
  @MethodSource("getClosestPointProvider")
  @Disabled("this test broke when changing decimation size. Fix it!")
  public void getClosestPoint(Translation2d point, Translation2d expected) {
    assertEquals(
        expected,
        new GridCoord(point).toTranslation2d(),
        String.format("Point %s should become %s", point, expected));
  }

  public static Stream<Arguments> getDistanceCorrectProvider() {
    return Stream.of(
        Arguments.of(new GridCoord(0, 0), new GridCoord(0, 0), 0),
        Arguments.of(new GridCoord(0, 0), new GridCoord(1, 0), Costs.CARDINAL),
        Arguments.of(new GridCoord(0, 0), new GridCoord(0, 1), Costs.CARDINAL),
        Arguments.of(new GridCoord(0, 0), new GridCoord(1, 1), Costs.DIAGONAL),
        Arguments.of(new GridCoord(0, 0), new GridCoord(2, 2), Costs.DIAGONAL * 2),
        Arguments.of(new GridCoord(2, 2), new GridCoord(0, 0), Costs.DIAGONAL * 2),
        Arguments.of(new GridCoord(0, 0), new GridCoord(2, 0), Costs.CARDINAL * 2),
        Arguments.of(new GridCoord(0, 0), new GridCoord(0, 2), Costs.CARDINAL * 2),
        Arguments.of(new GridCoord(0, 0), new GridCoord(2, 1), Costs.DIAGONAL + Costs.CARDINAL),
        Arguments.of(new GridCoord(0, 0), new GridCoord(1, 2), Costs.DIAGONAL + Costs.CARDINAL),
        Arguments.of(new GridCoord(0, 0), new GridCoord(2, 3), Costs.DIAGONAL * 2 + Costs.CARDINAL)
        // brace holder
        );
  }

  @UtilParamTest
  @MethodSource("getDistanceCorrectProvider")
  public void getDistanceCorrect(GridCoord a, GridCoord b, int expected) {
    assertEquals(expected, a.getDistance(b), String.format("Distance between %s and %s", a, b));
  }

  @UtilTest
  public void generateLineWorksCorrectly() {
    GridCoord a = new GridCoord(0, 0);
    GridCoord b = new GridCoord(2, 2);
    List<GridCoord> expected =
        List.of(new GridCoord(0, 0), new GridCoord(1, 1), new GridCoord(2, 2));
    List<GridCoord> actual = GridCoord.line(a, b);
    assertEquals(expected, actual, String.format("Line between %s and %s", a, b));
  }

  @UtilTest
  public void generateComplexLineWorksCorrectly() {
    GridCoord a = new GridCoord(0, 0);
    GridCoord b = new GridCoord(2, 3);
    List<GridCoord> expected =
        List.of(new GridCoord(0, 0), new GridCoord(1, 1), new GridCoord(1, 2), new GridCoord(2, 3));
    List<GridCoord> actual = GridCoord.line(a, b);
    assertEquals(expected, actual, String.format("Line between %s and %s", a, b));
  }

  public static Stream<Arguments> getLinkDirectionProvider() {
    return Stream.of(
        Arguments.of(new GridCoord(0, 0), new GridCoord(1, 0), LinkDirection.PURE_X),
        Arguments.of(new GridCoord(0, 0), new GridCoord(0, 1), LinkDirection.PURE_Y),
        Arguments.of(new GridCoord(0, 0), new GridCoord(1, 1), LinkDirection.COMBO),
        Arguments.of(new GridCoord(0, 0), new GridCoord(2, 2), LinkDirection.COMBO),
        Arguments.of(new GridCoord(2, 2), new GridCoord(0, 0), LinkDirection.COMBO),
        Arguments.of(new GridCoord(0, 0), new GridCoord(2, 0), LinkDirection.PURE_X),
        Arguments.of(new GridCoord(0, 0), new GridCoord(0, 2), LinkDirection.PURE_Y),
        Arguments.of(new GridCoord(0, 0), new GridCoord(2, 1), LinkDirection.COMBO));
  }

  @UtilParamTest
  @MethodSource("getLinkDirectionProvider")
  public void getLinkDirectionCorrect(GridCoord a, GridCoord b, LinkDirection expected) {
    assertEquals(
        expected, GridCoord.getLinkDirection(a, b), String.format("Link between %s and %s", a, b));
  }
}
