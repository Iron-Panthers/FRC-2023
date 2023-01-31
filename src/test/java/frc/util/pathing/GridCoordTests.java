package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilParamTest;
import java.util.stream.Stream;
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
  public void getClosestPoint(Translation2d point, Translation2d expected) {
    assertEquals(
        expected,
        new GridCoord(point).toTranslation2d(),
        String.format("Point %s should become %s", point, expected));
  }
}
