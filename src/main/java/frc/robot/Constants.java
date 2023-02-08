// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.util.MacUtil.IS_COMP_BOT;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

@SuppressWarnings("java:S1118")
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Drive {
    // max voltage delivered to drivebase
    // supposedly useful to limit speed for testing
    public static final double MAX_VOLTAGE = 12.0;
    // maximum velocity
    // FIXME measure this value experimentally
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0 // falcon 500 free speed rpm
            / 60.0
            * SdsModuleConfigurations.MK4_L2.getDriveReduction()
            * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
            * Math.PI;
    // theoretical value
    // FIXME measure and validate experimentally
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0)
            * .5;

    /** the maximum amount of angular error pid loops will tolerate for rotation */
    public static final double ANGULAR_ERROR = 1.0;
    /** the minimum magnitude of the right stick for it to be used as a new rotation angle */
    public static final double ROTATE_VECTOR_MAGNITUDE = .7;

    public static final class Dims {
      // FIXME validate with hardware
      public static final double TRACKWIDTH_METERS =
          .5207; // 20.5 inches (source: cad) converted to meters
      public static final double WHEELBASE_METERS = TRACKWIDTH_METERS; // robot is square
    }

    public static final class Modules {
      public static final class FrontRight { // Module 1
        public static final int DRIVE_MOTOR = 4;
        public static final int STEER_MOTOR = 3;
        public static final int STEER_ENCODER = 24;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(40.078) // comp bot offset
                : -Math.toRadians(39.462890); // practice bot offset
      }

      public static final class FrontLeft { // Module 2
        public static final int DRIVE_MOTOR = 11;
        public static final int STEER_MOTOR = 10;
        public static final int STEER_ENCODER = 25;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(39.64 + 180) // comp bot offset
                : -Math.toRadians(222.7148); // practice bot offset
      }

      public static final class BackLeft { // Module 3
        public static final int DRIVE_MOTOR = 13;
        public static final int STEER_MOTOR = 12;
        public static final int STEER_ENCODER = 26;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(309.99 + 180) // comp bot offset
                : -Math.toRadians(129.63867); // practice bot offset
      }

      public static final class BackRight { // Module 4
        public static final int DRIVE_MOTOR = 2;
        public static final int STEER_MOTOR = 1;
        public static final int STEER_ENCODER = 27;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(61.26) // comp bot offset
                : -Math.toRadians(61.3476); // practice bot offset
      }
    }
  }

  public static final class Arm {
    public static final class Ports {
      public static final int ARM_MOTOR_PORT = 14;
      public static final int TELESCOPING_MOTOR_PORT = 16; // TODO: find CAN ID
      public static final int ENCODER_PORT = 28;
    }

    public static final double GRAVITY_CONTROL_PERCENT = .06; // FIXME: Real value needed

    public static final int ANGULAR_OFFSET = 0; // FIXME: Real value needed

    public static final class Setpoints {
      public static final int STARTING_ANGLE = 0;
      public static final int FORWARD_ANGLE = 90;
      public static final int BACKWARD_ANGLE = -90;
      public static final double MAX_EXTENSION = 12; // FIXME: find accurate value
      public static final double MIN_EXTENSION = 0;
    }
    
    public static final int TICKS = 2048;
    public static final int TELESCOPING_ARM_GEAR_RATIO  = 3;
    public static final double SPOOL_CIRCUMFERENCE = 1.5 * Math.PI;

    public static final int ANGLE_THRESHOLD = 40; // FIXME: real value needed
    public static final int UPPER_ANGLE_LIMIT = 150; // FIXME: real value needed

  }

  public static final class Vision {
    public static final double LIMELIGHT_CLUSTER_HEIGHT = 0.3048;

    public static final class FrontCam {
      public static final String NAME = "frontCam";
      /** Cam mounted facing forward, centered, at the back of the robot */
      public static final Transform3d ROBOT_TO_CAM =
          new Transform3d(
              new Translation3d(-0.2248, 0, LIMELIGHT_CLUSTER_HEIGHT), new Rotation3d(0, 0, 0));
    }

    public static final class BackCam {
      public static final String NAME = "backCam";
      /** Cam mounted facing backward, centered, at the back of the robot */
      public static final Transform3d ROBOT_TO_CAM =
          new Transform3d(
              new Translation3d(-0.301, 0, LIMELIGHT_CLUSTER_HEIGHT),
              new Rotation3d(0, 0, Math.PI));
    }
  }

  public static final class PoseEstimator {
    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS =
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                0.1, // x
                0.1, // y
                0.1 // theta
                );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                2, // x
                2, // y
                2 * Math.PI // theta
                );
    public static final double CAMERA_CAPTURE_LATENCY_FUDGE_MS = 11;

    /** about one inch */
    public static final double DRIVE_TO_POSE_XY_ERROR_MARGIN_METERS = .05;

    public static final double DRIVE_TO_POSE_THETA_ERROR_MARGIN_DEGREES = 2;
  }

}
