// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Util {
  private Util() {}

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonZero(double a, double epsilon) {
    return epsilonEquals(a, 0, epsilon);
  }

  /**
   * Modulo on a negative number returns a negative number. This function always returns positive
   * numbers, even for negative angles, without rotation.
   *
   * @param degrees
   * @return normalized degrees
   */
  public static double normalizeDegrees(double degrees) {
    return (degrees % 360 + 360) % 360;
  }

  /**
   * This function finds the degree difference between angles, the shortest path. Useful for pid
   * control of drivebase rotation. Returns a value between -180 and 180 on a 360 reference field.
   * Values greater than 180 won't exist, because the other direction is shorter.
   *
   * @param currentAngle Current Angle Degrees
   * @param newAngle Target Angle Degrees
   * @return Shortest angular difference in degrees
   */
  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    double a = normalizeDegrees(currentAngle - newAngle);
    double b = normalizeDegrees(newAngle - currentAngle);
    return a < b ? a : -b;
  }

  public static double relativeAngularDifference(Rotation2d currentAngle, double newAngle) {
    return relativeAngularDifference(currentAngle.getDegrees(), newAngle);
  }

  public static double relativeAngularDifference(Rotation2d currentAngle, Rotation2d newAngle) {
    return relativeAngularDifference(currentAngle.getDegrees(), newAngle.getDegrees());
  }

  /**
   * turn x and y of a vector to a [0, 360] angle
   *
   * @param x x value of vector
   * @param y y value of vector
   * @return [0, 360] mapped angle of vector
   */
  public static double vectorToAngle(double x, double y) {
    double angle = Math.atan2(y, x);
    return (angle * (180 / Math.PI) + 360) % 360;
  }

  /**
   * snap an angle to the closest angle in an array of angles
   *
   * @param angle angle to be snapped
   * @param snaps array of angles to snap to
   * @return closest angle in snap array
   */
  public static double angleSnap(double angle, double[] snaps) {
    double closest = snaps[0];
    for (double snap : snaps) {
      if (Math.abs(relativeAngularDifference(angle, snap))
          < Math.abs(relativeAngularDifference(angle, closest))) {
        closest = snap;
      }
    }
    return closest;
  }

  public static double vectorMagnitude(double x, double y) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  public static double getVelocity(ChassisSpeeds chassisSpeeds) {
    return Math.sqrt(
        Math.pow(chassisSpeeds.vxMetersPerSecond, 2)
            + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
  }

  /**
   * Solve for the translation velocity of a robot given its frame relative chassis speeds and
   * heading. Does the opposite of {@link ChassisSpeeds#fromFieldRelativeSpeeds}.
   *
   * @param chassisSpeeds
   * @param robotAngle
   * @return
   */
  public static Translation2d getTranslationVelocity(
      ChassisSpeeds chassisSpeeds, Rotation2d robotAngle) {
    // the from field relative speeds impl
    /*
    public static ChassisSpeeds fromFieldRelativeSpeeds(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        Rotation2d robotAngle) {
      return new ChassisSpeeds(
          vxMetersPerSecond * robotAngle.getCos() + vyMetersPerSecond * robotAngle.getSin(),
          -vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(),
          omegaRadiansPerSecond);
    }
    */

    return new Translation2d(
        chassisSpeeds.vxMetersPerSecond * robotAngle.getCos()
            - chassisSpeeds.vyMetersPerSecond * robotAngle.getSin(),
        chassisSpeeds.vxMetersPerSecond * robotAngle.getSin()
            + chassisSpeeds.vyMetersPerSecond * robotAngle.getCos());
  }
}
