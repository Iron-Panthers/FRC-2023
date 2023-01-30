package frc.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import au.com.origin.snapshots.Expect;
import au.com.origin.snapshots.junit5.SnapshotExtension;
import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilParamTest;
import java.util.stream.Stream;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

@ExtendWith({SnapshotExtension.class})
public class RubenManueverGeneratorTests {

  private static Translation2d coord(double x, double y) {
    return new Translation2d(x, y);
  }

  public static Stream<Arguments> getClosestPointProvider() {
    return Stream.of(
        Arguments.of(coord(2.001, 2), coord(2, 2)),
        Arguments.of(coord(3.12, 2.01), coord(3.1, 2)),
        Arguments.of(coord(3.12, 2.06), coord(3.1, 2.1)),
        Arguments.of(coord(3.12, -.01), coord(3.1, 0))
        // brace holder
        );
  }

  @UtilParamTest
  @MethodSource("getClosestPointProvider")
  public void getClosestPoint(Translation2d point, Translation2d expected) {
    assertEquals(expected, RubenManueverGenerator.getClosestPoint(point));
  }

  public static Stream<Arguments> findFullPathMatchesSnapshotProvider() {
    return Stream.of(
        Arguments.of(coord(1, 1), coord(7, 7))

        // load bearing comment (hold the final brace)
        );
  }

  private Expect expect;

  @UtilParamTest
  @MethodSource("findFullPathMatchesSnapshotProvider")
  public void findFullPathMatchesSnapshot(Translation2d start, Translation2d end) {
    RubenManueverGenerator rubenManueverGenerator = new RubenManueverGenerator();

    var path =
        rubenManueverGenerator.findFullPath(
            RubenManueverGenerator.getClosestPoint(start),
            RubenManueverGenerator.getClosestPoint(end));

    expect
        .scenario(String.format("%s -> %s", start.toString(), end.toString()))
        .toMatchSnapshot(path);
  }
}
