package frc.util.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AlliancePose2d {
  private final Pose2d bluePose;
  private final Pose2d redPose;

  public AlliancePose2d(Pose2d bluePose) {
    this.bluePose = bluePose;
    this.redPose = PoseInversionUtility.findRedPose(bluePose);
  }

  /** Convenience constructor for creating an AlliancePose2d with the pose2d constructor. */
  public AlliancePose2d(double x, double y, Rotation2d rotation) {
    this(new Pose2d(x, y, rotation));
  }

  public Pose2d get() {
    var alliance = DriverStation.getAlliance();
    return switch (alliance) {
      case Blue -> bluePose;
      case Red -> redPose;
      case Invalid -> {
        System.out.println("Invalid alliance, defaulting to blue");
        yield bluePose;
      }
      default -> {
        System.out.printf("Unknown alliance %s, defaulting to blue", alliance);
        yield bluePose;
      }
    };
  }
}
