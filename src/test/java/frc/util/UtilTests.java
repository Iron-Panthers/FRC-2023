package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.UtilParamTest;
import frc.UtilTest;
import java.util.stream.Stream;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

public class UtilTests {
  @UtilTest
  public void epsilonReturnsTrueWhenEqual() {
    assertTrue(Util.epsilonEquals(2, 2.1, .2));
    assertTrue(Util.epsilonEquals(2, 2.1, .1));
  }

  @UtilTest
  public void epsilonReturnsFalseWhenUnequal() {
    assertFalse(Util.epsilonEquals(2, 2.1, 1e-2));
    assertFalse(Util.epsilonEquals(5, 2, 1));
  }

  @UtilTest
  public void relativeAngularDifferenceZerosOnEquivalent() {
    assertEquals(0, Util.relativeAngularDifference(100, 100), 1e-9);
    assertEquals(0, Util.relativeAngularDifference(0, 360 * 2), 1e-9);
    assertEquals(0, Util.relativeAngularDifference(0, 0), 1e-9);
  }

  @UtilTest
  public void relativeAngularDifferenceTakesShortestPath() {
    assertEquals(-100, Util.relativeAngularDifference(0, 100));
    assertEquals(-100, Util.relativeAngularDifference(360, 360 + 100));
    assertEquals(60, Util.relativeAngularDifference(360, 360 + 300));
    assertEquals(179, Util.relativeAngularDifference(0, 181));
    assertEquals(-170, Util.relativeAngularDifference(0, 170));
    assertEquals(-90, Util.relativeAngularDifference(270, 360));
    assertEquals(-90, Util.relativeAngularDifference(270, 0));
  }

  public static Stream<Arguments> relativeAngularDifferenceParams() {
    return Stream.of(
        Arguments.of(0, 0, 0),
        Arguments.of(1, 0, 1),
        Arguments.of(359, 0, -1),
        Arguments.of(1, 30, -29),
        Arguments.of(359, 30, -31),
        Arguments.of(1, 690, 31));
  }

  // parameterized test
  @UtilParamTest
  @MethodSource("relativeAngularDifferenceParams")
  public void relativeAngleHandlesRollover(
      double currentAngle, double newAngle, double difference) {
    assertEquals(Util.relativeAngularDifference(currentAngle, newAngle), difference, 1e-9);
  }

  @UtilTest
  public void vectorToAngleCorrect() {
    assertEquals(0, Util.vectorToAngle(0, 0));
    assertEquals(45, Util.vectorToAngle(1, 1));
    assertEquals(270, Util.vectorToAngle(0, -5));
    assertEquals(359, Util.vectorToAngle(1, -.015), .2);
  }

  @UtilTest
  public void angleSnapCorrect() {
    double[] snaps1 = {90, 270, 350};
    double[] snaps2 = {0, 45, 90, 135, 180, 225, 270, 315};
    assertEquals(0, Util.angleSnap(0, snaps2), "exact match snaps");
    assertEquals(45, Util.angleSnap(56.5, snaps2));
    assertEquals(270, Util.angleSnap(265, snaps1));
    assertEquals(270, Util.angleSnap(290, snaps1));
    assertEquals(350, Util.angleSnap(360, snaps1));
    assertEquals(0, Util.angleSnap(5, snaps2), "close match snaps down");
    assertEquals(
        0, Util.angleSnap(355, snaps2), "snaps to closer angle, even if its on a different level");
  }

  public static Stream<Arguments> vectorMagnitudeProvider() {
    double sqrt2 = Math.sqrt(2);
    double sqrt3 = Math.sqrt(3);
    return Stream.of(
        Arguments.of(1, 0, 1),
        Arguments.of(sqrt2 / 2, sqrt2 / 2, 1),
        Arguments.of(sqrt3 / 2, .5, 1),
        Arguments.of(-sqrt2 / 2, sqrt2 / 2, 1),
        Arguments.of(-sqrt2 / 2, -sqrt2 / 2, 1),
        Arguments.of(sqrt2, sqrt2, 2),
        Arguments.of(10, 0, 10));
  }

  @UtilParamTest
  @MethodSource("vectorMagnitudeProvider")
  public void vectorMagnitudeCorrect(double x, double y, double magnitude) {
    assertEquals(
        magnitude,
        Util.vectorMagnitude(x, y),
        1e-9,
        String.format("||<%s, %s>|| == %s", x, y, magnitude));
  }

  public static Stream<Arguments> normalizeDegreeProvider() {
    return Stream.of(
        Arguments.of(0, 0),
        Arguments.of(360, 0),
        Arguments.of(720, 0),
        Arguments.of(90, 90),
        Arguments.of(450, 90),
        Arguments.of(270, 270),
        Arguments.of(-90, 270),
        Arguments.of(-270, 90),
        Arguments.of(-360, 0),
        Arguments.of(-720, 0),
        // tests for non 90 degree angles
        Arguments.of(45, 45),
        Arguments.of(135, 135),
        Arguments.of(225, 225),
        Arguments.of(796, 76),
        Arguments.of(-43, 317));
  }

  @UtilParamTest
  @MethodSource("normalizeDegreeProvider")
  public void normalizeDegreeCorrect(double degrees, double normalized) {
    assertEquals(
        normalized,
        Util.normalizeDegrees(degrees),
        1e-9,
        String.format("%s normalized to %s", degrees, normalized));
  }

  public static Stream<Arguments> getTranslationVelocityProvider() {
    return Stream.of(
        Arguments.of(new Translation2d(3, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0)),
        Arguments.of(new Translation2d(5, Rotation2d.fromDegrees(30)), Rotation2d.fromDegrees(0)),
        Arguments.of(new Translation2d(5, Rotation2d.fromDegrees(235)), Rotation2d.fromDegrees(0)),
        Arguments.of(new Translation2d(5, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(30)),
        Arguments.of(new Translation2d(5, Rotation2d.fromDegrees(20)), Rotation2d.fromDegrees(235)),
        Arguments.of(
            new Translation2d(-2.5, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(60))
        // hold this brace here lol
        );
  }

  @UtilParamTest
  @MethodSource("getTranslationVelocityProvider")
  public void getTranslationVelocityCorrect(Translation2d translation, Rotation2d robotAngle) {
    var chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), 0, robotAngle);
    var translationVelocity = Util.getTranslationVelocity(chassisSpeeds, robotAngle);
    assertEquals(
        translation, translationVelocity, String.format("translation velocity is %s", translation));
  }
}
