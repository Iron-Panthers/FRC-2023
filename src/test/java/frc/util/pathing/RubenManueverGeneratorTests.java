package frc.util.pathing;

import au.com.origin.snapshots.junit5.SnapshotExtension;
import edu.wpi.first.math.geometry.Translation2d;
import frc.UtilParamTest;
import java.util.stream.Stream;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

@ExtendWith({SnapshotExtension.class})
public class RubenManueverGeneratorTests {

  public static Stream<Arguments> rubenManueverFindFullPathMatchesSnapshotProvider() {
    return Stream.of(Arguments.of());
  }

  @UtilParamTest
  @MethodSource("rubenManueverFindFullPathMatchesSnapshotProvider")
  public static void rubenManueverFindFullPathMatchesSnapshot(
      Translation2d start, Translation2d end) {}
}
