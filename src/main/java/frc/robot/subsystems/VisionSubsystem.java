// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.PoseEstimator;
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
      cameraStatusList.addBoolean(this.camera.getName(), this.camera::isConnected);
    }

    Pair<PhotonCamera, Transform3d> getAsPair() {
      return new Pair<PhotonCamera, Transform3d>(camera, robotToCam);
    }
  }

  private final ShuffleboardLayout cameraStatusList =
      Shuffleboard.getTab("DriverView")
          .getLayout("photonCameras", BuiltInLayouts.kList)
          .withPosition(11, 0)
          .withSize(2, 3);

  private final VisionSource frontCam =
      new VisionSource(new PhotonCamera(Vision.FrontCam.NAME), Vision.FrontCam.ROBOT_TO_CAM);
  private final VisionSource backCam =
      new VisionSource(new PhotonCamera(Vision.BackCam.NAME), Vision.BackCam.ROBOT_TO_CAM);

  private final AprilTagFieldLayout fieldLayout;

  RobotPoseEstimator poseEstimator;

  static final double TEST_SPACE_WIDTH = 2.5;
  static final double TEST_SPACE_LENGTH = 5;

  private double lastDetection = 0;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

    // loading the 2023 field arrangement
    try {
      fieldLayout = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      System.err.println("Failed to load field layout.");
      throw e;
    }

    // Create a list of cameras to use for pose estimation
    List<Pair<PhotonCamera, Transform3d>> cameras = new ArrayList<>();
    cameras.add(frontCam.getAsPair());
    cameras.add(backCam.getAsPair());

    poseEstimator =
        new RobotPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameras);

    cameraStatusList.addString(
        "time since apriltag detection",
        () -> String.format("%3.0f seconds", Timer.getFPGATimestamp() - lastDetection));
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

    lastDetection = Timer.getFPGATimestamp();

    return Optional.of(
        new Pair<Pose2d, Double>(
            resultSome.getFirst().toPose2d(),
            currentTime
                - ((resultSome.getSecond() + PoseEstimator.CAMERA_CAPTURE_LATENCY_FUDGE_MS)
                    / 1000)));
  }
}
