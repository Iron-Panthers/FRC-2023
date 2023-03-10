package frc.util.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AlliancePose2d {
  private final Pose2d bluePose;
  private final Pose2d redPose;

  public AlliancePose2d(Pose2d bluePose) {
    this.bluePose = bluePose;
    this.redPose =
        new Pose2d(
            bluePose.getTranslation().getX(),
            FieldObstructionMap.FIELD_HEIGHT - bluePose.getTranslation().getY(),
            bluePose.getRotation().times(-1));
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
