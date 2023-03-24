package frc.util.pathing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

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

  public static List<Supplier<PathPlannerTrajectory>> loadPathGroup(
      String pathName, double maxVelocityMetersPerSecond, double maxAccelerationMetersPerSecondSq) {
    List<PathPlannerTrajectory> bluePath =
        PathPlanner.loadPathGroup(
            pathName, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    List<PathPlannerTrajectory> redPath =
        PathPlanner.loadPathGroup(
            pathName + ".path.mirror",
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSq);

    return IntStream.range(0, bluePath.size())
        .mapToObj(
            i ->
                (Supplier<PathPlannerTrajectory>)
                    () ->
                        DriverStation.getAlliance() == Alliance.Red
                            ? redPath.get(i)
                            : bluePath.get(i))
        .collect(Collectors.toList());
  }

  public static List<Supplier<PathPlannerTrajectory>> loadPathGroup(
      String pathName, PathConstraints constraint, PathConstraints... constraints) {
    List<PathPlannerTrajectory> bluePath =
        PathPlanner.loadPathGroup(pathName, constraint, constraints);

    List<PathPlannerTrajectory> redPath =
        PathPlanner.loadPathGroup(pathName + ".path.mirror", constraint, constraints);

    return IntStream.range(0, bluePath.size())
        .mapToObj(
            i ->
                (Supplier<PathPlannerTrajectory>)
                    () ->
                        DriverStation.getAlliance() == Alliance.Red
                            ? redPath.get(i)
                            : bluePath.get(i))
        .collect(Collectors.toList());
  }
}
