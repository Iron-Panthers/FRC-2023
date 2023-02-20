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
import frc.robot.subsystems.OuttakeSubsystem.OuttakeDetails;
import java.util.Optional;

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

    /*
     module layout:
        ┌──────
     ┌─►│#   ##steer motor
     │  │  ##cancoder
     │  │##drive motor
     module number

     steer is always left
     from corner perspective

     robot visualization:
    ┌──────────────────────┐
    │2   10          04   1│
    │  25              24  │
    │11     S      D     03│
    │     D          S     │
    │                      │
    │                      │
    │     S          D     │
    │       D      S       │
    │12    ┌────────┐    02│
    │  26  │        │  27  │
    │3   13│  batt  │01   4│
    └──────┴───┬┬───┴──────┘
               ││
               ││
               ▼▼
         software front
     */

    public static final class Modules {
      public static final class FrontRight { // Module 1
        public static final int DRIVE_MOTOR = 4;
        public static final int STEER_MOTOR = 3;
        public static final int STEER_ENCODER = 24;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(8.07400 + 180) // comp bot offset
                : -Math.toRadians(39.462890); // practice bot offset
      }

      public static final class FrontLeft { // Module 2
        public static final int DRIVE_MOTOR = 11;
        public static final int STEER_MOTOR = 10;
        public static final int STEER_ENCODER = 25;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(274.562 + 180) // comp bot offset
                : -Math.toRadians(222.7148); // practice bot offset
      }

      public static final class BackLeft { // Module 3
        public static final int DRIVE_MOTOR = 13;
        public static final int STEER_MOTOR = 12;
        public static final int STEER_ENCODER = 26;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(225.082 + 180) // comp bot offset
                : -Math.toRadians(129.63867); // practice bot offset
      }

      public static final class BackRight { // Module 4
        public static final int DRIVE_MOTOR = 2;
        public static final int STEER_MOTOR = 1;
        public static final int STEER_ENCODER = 27;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(335.124 + 180) // comp bot offset
                : -Math.toRadians(61.3476); // practice bot offset
      }
    }
  }

  public static final class Arm {
    public static final class Ports {
      public static final int ARM_MOTOR_PORT = 16;
      public static final int TELESCOPING_MOTOR_PORT = 17; // TODO: find CAN ID
      public static final int ENCODER_PORT = 28;
    }

    public static final double GRAVITY_CONTROL_PERCENT = 0.07;

    public static final double ANGULAR_OFFSET = 8;

    public static final class Setpoints {
      public static final class ScoreLow {
        public static final int ANGLE = 40;
        public static final double EXTENSION = Extensions.MAX_EXTENSION;
      }

      public static final class ScoreMid {
        public static final int ANGLE = 90;
        public static final double EXTENSION = Extensions.MAX_EXTENSION;
      }

      public static final class ScoreHigh {
        public static final int ANGLE = 110;
        public static final double EXTENSION = Extensions.MAX_EXTENSION;
      }

      public static final class GroundIntake {
        public static final int ANGLE = 40;
        public static final double EXTENSION = Extensions.MAX_EXTENSION;
      }

      public static final class ShelfIntake {
        public static final int ANGLE = 90;
        public static final double EXTENSION = Extensions.MAX_EXTENSION;
      }

      public static final class Angles {
        public static final int STARTING_ANGLE = 0;
        public static final int FORWARD_ANGLE = 90;
        public static final int BACKWARD_ANGLE = -90;
        public static final int TEST_ANGLE = 45;
      }

      public static final class Extensions {
        public static final double MAX_EXTENSION = 10;
        public static final double MIN_EXTENSION = 0;
      }
    }

    public static final double EXTENSION_STATORLIMIT = 80;

    public static final double ZERO_RETRACTION_PERCENT = -0.4; // FIXME
    public static final int TICKS = 2048;
    public static final int TELESCOPING_ARM_GEAR_RATIO = 3;
    public static final double SPOOL_CIRCUMFERENCE = 1.5 * Math.PI;

    public static final class Thresholds {
      /**
       * These thresholds, unless otherwise specified in a doc comment, apply to the positive and
       * negative sign of their angle in degrees
       */
      public static final class Angles {
        public static final double BACKWARD_UNSAFE_EXTENSION_ANGLE_THRESHOLD =
            -40; // FIXME: real value needed
        public static final double FORWARD_UNSAFE_EXTENSION_ANGLE_THRESHOLD =
            20; // FIXME: real value needed
        public static final double UPPER_ANGLE_LIMIT = 100; // FIXME: real value needed
      }

      public static final class Extensions {
        /**
         * The amount of additional extension from min extension to treat as fully retracted for
         * safety purposes
         */
        public static final double FULLY_RETRACTED_INCHES_THRESHOLD = 1;
      }
    }
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

  public static final class Outtake {
    public static final class Ports {
      public static final int OUTTAKE_MOTOR = 17; // Placeholder value
      public static final int OUTTAKE_ENCODER = 0; // PLaceholder value
    }

    public static final int OPEN_ANGLE = 500;
    public static final int CLAMP_ANGLE = 0;

    public static final class OuttakeModes {
      public static final OuttakeDetails HOLD = new OuttakeDetails(0.1, Optional.empty());
      public static final OuttakeDetails INTAKE =
          new OuttakeDetails(0.7, Optional.of(new OuttakeDetails.StatorLimit(75, true)));
      public static final OuttakeDetails OUTTAKE =
          new OuttakeDetails(-0.2, Optional.of(new OuttakeDetails.StatorLimit(10, false)));
      public static final OuttakeDetails OFF = new OuttakeDetails(0.0, Optional.empty());
    }

    // Thinking of using these to plug into the stator limits above...?
    // Better readability?
    private static final class StatorCurrents {
      // FIXME find real value using glass
      public static final double OPENING_FINISH = 20;
      public static final double ENDING_FINISH = 80;
    }
  }
}
