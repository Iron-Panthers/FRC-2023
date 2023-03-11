package frc.util.pathing;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public class LoadMirrorPath {
  private LoadMirrorPath() {}

  public static Supplier<PathPlannerTrajectory> loadPath(
      String pathName, double maxVelocityMetersPerSecond, double maxAccelerationMetersPerSecondSq) {
    PathPlannerTrajectory bluePath =
        PathPlanner.loadPath(
            pathName, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    PathPlannerTrajectory redPath =
        PathPlanner.loadPath(
            pathName + ".path.mirror",
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSq);

    return () -> DriverStation.getAlliance() == Alliance.Red ? redPath : bluePath;
  }
}
