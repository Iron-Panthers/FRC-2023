package frc.util.pathing;

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

  public static Stream<Arguments> rubenManueverFindFullPathMatchesSnapshotProvider() {
    return Stream.of(Arguments.of(coord(1, 1), coord(7, 7)));
  }

  private Expect expect;

  @UtilParamTest
  @MethodSource("rubenManueverFindFullPathMatchesSnapshotProvider")
  public void rubenManueverFindFullPathMatchesSnapshot(Translation2d start, Translation2d end) {
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
