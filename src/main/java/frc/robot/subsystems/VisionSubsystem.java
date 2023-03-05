// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.PoseEstimator;
import frc.robot.Constants.Vision;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionSubsystem {
  /** If shuffleboard should be used--important for unit testing. */
  private static boolean useShuffleboard = true;

  private final ShuffleboardLayout cameraStatusList =
      Shuffleboard.getTab("DriverView")
          .getLayout("photonCameras", BuiltInLayouts.kList)
          .withPosition(11, 0)
          .withSize(2, 3);

  private final List<PhotonPoseEstimator> estimators = new ArrayList<>();

  private AprilTagFieldLayout fieldLayout;

  private double lastDetection = 0;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    // loading the 2023 field arrangement
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      System.err.println("Failed to load field layout.");
      e.printStackTrace();
      return;
    }

    for (Vision.VisionSource visionSource : Vision.VISION_SOURCES) {
      var camera = new PhotonCamera(visionSource.name());
      var estimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
              camera,
              visionSource.robotToCamera());
      estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
      cameraStatusList.addBoolean(visionSource.name(), camera::isConnected);
    }

    if (useShuffleboard)
      cameraStatusList.addString(
          "time since apriltag detection",
          () -> String.format("%3.0f seconds", Timer.getFPGATimestamp() - lastDetection));
  }

  public static record VisionMeasurement(
      EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {}

  public List<VisionMeasurement> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (fieldLayout == null) {
      return List.of();
    }

    List<VisionMeasurement> estimations = new ArrayList<>();

    boolean foundTag = false;

    for (PhotonPoseEstimator estimator : estimators) {
      estimator.setReferencePose(prevEstimatedRobotPose);
      var optEstimation = estimator.update();
      if (optEstimation.isEmpty()) continue;
      foundTag = true;
      var estimation = optEstimation.get();
      estimations.add(
          new VisionMeasurement(estimation, PoseEstimator.VISION_MEASUREMENT_STANDARD_DEVIATIONS));
    }

    if (foundTag) {
      lastDetection = Timer.getFPGATimestamp();
    }

    return estimations;
  }
}
