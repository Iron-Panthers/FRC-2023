// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.Config;
import frc.robot.Constants.PoseEstimator;
import frc.robot.Constants.Vision;
import frc.util.CSV;
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
      estimators.add(estimator);
    }

    if (useShuffleboard)
      cameraStatusList.addString(
          "time since apriltag detection",
          () -> String.format("%3.0f seconds", Timer.getFPGATimestamp() - lastDetection));
  }

  record MeasurementRow(
      double realX,
      double realY,
      int tags,
      double avgDistance,
      double ambiguity,
      double estX,
      double estY,
      double estTheta) {}

  private final CSV<MeasurementRow> measurementCSV =
      Config.WRITE_APRILTAG_DATA
          ? new CSV<>(
              Config.APRILTAG_DATA_PATH,
              List.of(
                  CSV.column("realX", MeasurementRow::realX),
                  CSV.column("realY", MeasurementRow::realY),
                  CSV.column("tags", MeasurementRow::tags),
                  CSV.column("avgDistance", MeasurementRow::avgDistance),
                  CSV.column("ambiguity", MeasurementRow::ambiguity),
                  CSV.column("estX", MeasurementRow::estX),
                  CSV.column("estY", MeasurementRow::estY),
                  CSV.column("estTheta", MeasurementRow::estTheta)))
          : null;

  private void logMeasurement(int tags, double avgDistance, double ambiguity, Pose3d est) {
    if (!Config.WRITE_APRILTAG_DATA) return;

    measurementCSV.write(
        new MeasurementRow(
            Config.REAL_X,
            Config.REAL_Y,
            tags,
            avgDistance,
            ambiguity,
            est.toPose2d().getTranslation().getX(),
            est.toPose2d().getTranslation().getY(),
            est.toPose2d().getRotation().getRadians()));
  }

  public static record VisionMeasurement(
      EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {}

  public List<VisionMeasurement> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (fieldLayout == null) {
      return List.of();
    }

    List<VisionMeasurement> estimations = new ArrayList<>();

    for (PhotonPoseEstimator estimator : estimators) {
      estimator.setReferencePose(prevEstimatedRobotPose);
      var optEstimation = estimator.update();
      if (optEstimation.isEmpty()) continue;
      var estimation = optEstimation.get();
      double smallestDistance = Double.POSITIVE_INFINITY;
      for (var target : estimation.targetsUsed) {
        var t3d = target.getBestCameraToTarget();
        var distance =
            Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
        if (distance < smallestDistance) smallestDistance = distance;
      }
      double poseAmbiguityFactor =
          estimation.targetsUsed.size() != 1
              ? 1
              : Math.max(
                  1,
                  (estimation.targetsUsed.get(0).getPoseAmbiguity()
                          + PoseEstimator.POSE_AMBIGUITY_SHIFTER)
                      * PoseEstimator.POSE_AMBIGUITY_MULTIPLIER);
      double confidenceMultiplier =
          Math.max(
              1,
              (Math.max(
                          1,
                          Math.max(0, smallestDistance - PoseEstimator.NOISY_DISTANCE_METERS)
                              * PoseEstimator.DISTANCE_WEIGHT)
                      * poseAmbiguityFactor)
                  / (1
                      + ((estimation.targetsUsed.size() - 1) * PoseEstimator.TAG_PRESENCE_WEIGHT)));
      // System.out.println(
      //     String.format(
      //         "with %d tags at smallest distance %f and pose ambiguity factor %f, confidence
      // multiplier %f",
      //         estimation.targetsUsed.size(),
      //         smallestDistance,
      //         poseAmbiguityFactor,
      //         confidenceMultiplier));
      logMeasurement(
          estimation.targetsUsed.size(),
          smallestDistance,
          estimation.targetsUsed.get(0).getPoseAmbiguity(),
          estimation.estimatedPose);
      estimations.add(
          new VisionMeasurement(
              estimation,
              PoseEstimator.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier)));
    }

    if (!estimations.isEmpty()) {
      lastDetection = Timer.getFPGATimestamp();
    }

    return estimations;
  }
}
