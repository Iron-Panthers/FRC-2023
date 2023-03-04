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
import frc.robot.Constants.Drive.Dims;
import frc.robot.commands.ScoreCommand.ScoreStep;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.NetworkWatchdogSubsystem.IPv4;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem.OuttakeDetails;
import frc.robot.subsystems.RGBSubsystem.RGBColor;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeType;
import frc.util.NodeSelectorUtility.ScoreTypeIdentifier;
import frc.util.pathing.FieldObstructionMap;
import java.util.List;
import java.util.Map;
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

      public static final double BUMPER_WIDTH_METERS = .851;
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
                : -Math.toRadians(129.375 + 180); // practice bot offset
      }

      public static final class FrontLeft { // Module 2
        public static final int DRIVE_MOTOR = 11;
        public static final int STEER_MOTOR = 10;
        public static final int STEER_ENCODER = 25;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(274.562 + 180) // comp bot offset
                : -Math.toRadians(129.375 + 180); // practice bot offset
      }

      public static final class BackLeft { // Module 3
        public static final int DRIVE_MOTOR = 13;
        public static final int STEER_MOTOR = 12;
        public static final int STEER_ENCODER = 26;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(225.082 + 180) // comp bot offset
                : -Math.toRadians(307.793 + 180); // practice bot offset
      }

      public static final class BackRight { // Module 4
        public static final int DRIVE_MOTOR = 2;
        public static final int STEER_MOTOR = 1;
        public static final int STEER_ENCODER = 27;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(335.124 + 180) // comp bot offset
                : -Math.toRadians(241.963 + 180); // practice bot offset
      }
    }
  }

  public static final class Arm {
    public static final class Ports {
      public static final int ARM_MOTOR_PORT = 16;
      public static final int TELESCOPING_MOTOR_PORT = 17;
      public static final int ENCODER_PORT = 28;
    }

    public static final double GRAVITY_CONTROL_PERCENT = 0.07;

    public static final double ANGULAR_OFFSET = -8.75;

    public static final class Setpoints {

      public static final ArmState GROUND_INTAKE = new ArmState(-45, 19);

      public static final ArmState SHELF_INTAKE = new ArmState(95, 0);

      public static final ArmState STOWED = new ArmState(0, Arm.Setpoints.Extensions.MIN_EXTENSION);

      public static final class Extensions {
        public static final double MAX_EXTENSION = 20.7;
        public static final double MIN_EXTENSION = 0.4;
      }
    }

    public static final double EXTENSION_STATOR_LIMIT = 42;

    public static final double ZERO_RETRACTION_PERCENT = -0.14;
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
            -35; // FIXME: real value needed
        public static final double FORWARD_UNSAFE_EXTENSION_ANGLE_THRESHOLD =
            20; // FIXME: real value needed
        public static final double UPPER_ANGLE_LIMIT = 120;
        public static final double EPSILON = 5;
      }

      public static final class Extensions {
        /**
         * The amount of additional extension from min extension to treat as fully retracted for
         * safety purposes
         */
        public static final double FULLY_RETRACTED_INCHES_THRESHOLD = 1;

        public static final double EPSILON = .5;
      }
    }
  }

  public static final Map<ScoreTypeIdentifier, List<ScoreStep>> SCORE_STEP_MAP =
      Map.of(
          NodeType.CONE.atHeight(Height.HIGH),
              List.of(
                  new ScoreStep(new ArmState(115, Arm.Setpoints.Extensions.MIN_EXTENSION)),
                  new ScoreStep(new ArmState(115, Arm.Setpoints.Extensions.MAX_EXTENSION))
                      .canWaitHere(),
                  new ScoreStep(new ArmState(87, Arm.Setpoints.Extensions.MAX_EXTENSION))
                      .canWaitHere(),
                  new ScoreStep(
                      new ArmState(87, Arm.Setpoints.Extensions.MIN_EXTENSION),
                      OuttakeSubsystem.Modes.OUTTAKE)),
          NodeType.CONE.atHeight(Height.MID),
              List.of(
                  new ScoreStep(new ArmState(100, Arm.Setpoints.Extensions.MIN_EXTENSION)),
                  new ScoreStep(new ArmState(100, 4.8)).canWaitHere(),
                  new ScoreStep(new ArmState(75, 4.8)).canWaitHere(),
                  new ScoreStep(
                      new ArmState(80, Arm.Setpoints.Extensions.MIN_EXTENSION),
                      OuttakeSubsystem.Modes.OUTTAKE)));

  public static final class Vision {

    public static final class FrontCam {
      public static final String NAME = "frontCam";
      public static final Transform3d ROBOT_TO_CAM =
          new Transform3d(
              // 9.867 in to the right looking from behind the front of the robot
              // 7 inch forward from center
              // up 17.422 inches
              new Translation3d(
                  0.1778, // front/back
                  0.2506218, // left/right
                  0.4425188 // up/down
                  ),
              new Rotation3d(
                  0,
                  Math.toRadians(-11.5), // angle up/down
                  0));
    }

    public static final class BackCam {
      public static final String NAME = "backCam";
      public static final Transform3d ROBOT_TO_CAM =
          new Transform3d(
              // 9.867 in to the right looking from behind the front of the robot
              // 48.5 inches up
              // two inches forward
              new Translation3d(
                  0.0508, // front/back
                  -0.2506218, // left/right
                  1.2319 // up/down
                  ),
              new Rotation3d(0, Math.toRadians(17), Math.PI));
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
                .9, // x
                .9, // y
                .9 * Math.PI // theta
                );

    public static final double CAMERA_CAPTURE_LATENCY_FUDGE_MS = 11;

    /** about one inch */
    public static final double DRIVE_TO_POSE_XY_ERROR_MARGIN_METERS = .05;

    public static final double DRIVE_TO_POSE_THETA_ERROR_MARGIN_DEGREES = 2;
  }

  public static final class Pathing {
    /** The size in meters of a given cell for pathfinding */
    public static final double CELL_SIZE_METERS = 0.1;

    public static final int CELL_X_MAX =
        (int) Math.ceil(FieldObstructionMap.FIELD_LENGTH / Pathing.CELL_SIZE_METERS);
    public static final int CELL_Y_MAX =
        (int) Math.ceil(FieldObstructionMap.FIELD_HEIGHT / Pathing.CELL_SIZE_METERS);

    /**
     * this variable is badly named, it refers to half the width decimated to the cell grid. coords
     * that require going within this distance will be very expensive for pathfinding.
     */
    public static final int ROBOT_RADIUS_DANGER_CELLS =
        // using floor is not a bug, we want to be able to drive up to the edge of the cell if
        // needed. this might not work too hot for other robot sizes, but for our size down is much
        // more reasonable than up for .1m cells
        // adding one serves to reduce the risk of a spline clipping something
        (int) Math.floor((Dims.BUMPER_WIDTH_METERS / 2) / Pathing.CELL_SIZE_METERS) + 1;

    /**
     * grid coords that require going within this distance of field elements will be unavailable for
     * pathfinding. subtracting one serves to make this number accurate because we added one
     * earlier.
     */
    public static final int ROBOT_RADIUS_COLLISION_CELLS = ROBOT_RADIUS_DANGER_CELLS - 2;

    public static final double CRITICAL_POINT_DIVERGENCE_THRESHOLD = 6;

    public static final int PATHFINDING_HEURISTIC_CONSTANT = 1;

    public static final double RESPECT_CURRENT_VELOCITY_THRESHOLD_MS = .2;

    public static final double ANTICIPATED_PATH_SOLVE_TIME_SECONDS = 1;

    public static final class Costs {
      public static final int CARDINAL = 2;
      public static final int DIAGONAL = 3;
      public static final int DANGER_MULTIPLIER = 50;
    }
  }

  public static final class Outtake {
    public static final class Ports {
      public static final int OUTTAKE_MOTOR = 8; // Placeholder value
    }

    public static final class OuttakeModes {
      public static final OuttakeDetails HOLD =
          new OuttakeDetails(0.1, Optional.empty(), Optional.empty());

      public static final OuttakeDetails INTAKE =
          new OuttakeDetails(0.7, Optional.of(new OuttakeDetails.StatorLimit(75)), Optional.of(2d));

      public static final OuttakeDetails OUTTAKE =
          new OuttakeDetails(-0.2, Optional.empty(), Optional.of(2d));

      public static final OuttakeDetails OFF =
          new OuttakeDetails(0.0, Optional.empty(), Optional.empty());
    }
  }

  public static final class NetworkWatchdog {
    /** The IP addresses to ping for testing bridging, on the second vlan. */
    public static final List<IPv4> TEST_IP_ADDRESSES =
        List.of(IPv4.internal(17), IPv4.internal(18), IPv4.internal(19));

    /**
     * The number of ms (sleep delta using oshi system uptime) to wait before beginning to ping the
     * test IP.
     */
    public static final int BOOT_SCAN_DELAY_MS = 50_000;

    /** The number of seconds for ping to wait before giving up on reaching a device. */
    public static final int PING_TIMEOUT_SECONDS = 2;

    /** The number of ms to wait before retrying successful health checks. */
    public static final int HEALTHY_CHECK_INTERVAL_MS = 5_000;

    /**
     * The number of ms to leave the switching pdh port off before turning it back on as part of
     * rebooting the network switch.
     */
    public static final int REBOOT_DURATION_MS = 1_000;

    /**
     * The number of ms to wait before rerunning health checks after a failed check which triggered
     * switch reboot.
     */
    public static final int SWITCH_POWERCYCLE_SCAN_DELAY_MS = 25_000;
  }

  public static final class Lights {
    public static final int CANDLE_ID = 34;
    public static final int NUM_LEDS =
        91
            // 8 inside the candle
            + 8;

    public static final class Colors {
      public static final RGBColor YELLOW = new RGBColor(255, 107, 0);
      public static final RGBColor PURPLE = new RGBColor(127, 0, 127);
      public static final RGBColor RED = new RGBColor(255, 0, 0);
      public static final RGBColor BLUE = new RGBColor(0, 0, 255);
      public static final RGBColor PINK = new RGBColor(250, 35, 100);
      public static final RGBColor MINT = new RGBColor(55, 255, 50);
      public static final RGBColor TEAL = new RGBColor(0, 255, 255);
    }
  }
}
