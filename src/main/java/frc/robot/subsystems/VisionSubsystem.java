// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Vision;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class VisionSubsystem {
  class VisionSource {
    PhotonCamera camera;
    Transform3d robotToCam;

    public VisionSource(PhotonCamera camera, Transform3d robotToCam) {
      this.camera = camera;
      this.robotToCam = robotToCam;
    }
  }

  private final VisionSource frontCam =
      new VisionSource(new PhotonCamera(Vision.FrontCam.NAME), Vision.FrontCam.ROBOT_TO_CAM);
  private final VisionSource backCam =
      new VisionSource(new PhotonCamera(Vision.BackCam.NAME), Vision.BackCam.ROBOT_TO_CAM);

  RobotPoseEstimator poseEstimator;

  static final double TEST_SPACE_WIDTH = 10;
  static final double TEST_SPACE_LENGTH = 10;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    // Set up a test arena of two apriltags at the center of each driver station set
    final AprilTag tag18 =
        new AprilTag(
            18,
            new Pose3d(
                new Pose2d(
                    TEST_SPACE_LENGTH, TEST_SPACE_WIDTH / 2.0, Rotation2d.fromDegrees(180))));
    final AprilTag tag01 =
        new AprilTag(
            01, new Pose3d(new Pose2d(0.0, TEST_SPACE_WIDTH / 2.0, Rotation2d.fromDegrees(0.0))));
    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    atList.add(tag18);
    atList.add(tag01);

    // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
    AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, TEST_SPACE_LENGTH, TEST_SPACE_WIDTH);

    // Create a list of cameras to use for pose estimation
    List<Pair<PhotonCamera, Transform3d>> cameras = new ArrayList<>();
    cameras.add(new Pair<>(frontCam.camera, frontCam.robotToCam));
    cameras.add(new Pair<>(backCam.camera, backCam.robotToCam));

    poseEstimator = new RobotPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameras);
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
   *     of the observation in seconds. Assumes a planar field and the robot is always firmly on the
   *     ground
   */
  public Optional<Pair<Pose2d, Double>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = poseEstimator.update();

    // optional will be present but first can still be null!
    if (!result.isPresent() || result.get().getFirst() == null) return Optional.empty();

    var resultSome = result.get();

    return Optional.of(
        new Pair<Pose2d, Double>(
            resultSome.getFirst().toPose2d(), currentTime - resultSome.getSecond()));
  }
}
