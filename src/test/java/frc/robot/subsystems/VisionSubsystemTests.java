package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.RobotParamTest;
import frc.RobotTest;
import frc.robot.Constants.Vision;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.photonvision.PhotonCamera;

public class VisionSubsystemTests {

  private PhotonCamera mockFrontCamera;

  private VisionSubsystem.VisionSource mockFrontVisionSource;

  private PhotonCamera mockBackCamera;

  private VisionSubsystem.VisionSource mockBackVisionSource;

  private PhotonCamera mockLeftCamera;

  private VisionSubsystem.VisionSource mockLeftVisionSource;

  VisionSubsystem visionSubsystem;

  @BeforeEach
  public void setup() {
    // set useShuffleboard to false using reflection so that we don't try to use shuffleboard
    // in unit tests
    try {
      Field useShuffleboardField = VisionSubsystem.class.getDeclaredField("useShuffleboard");
      useShuffleboardField.setAccessible(true);
      useShuffleboardField.set(visionSubsystem, false);

    } catch (NoSuchFieldException | IllegalAccessException e) {
      e.printStackTrace();
    }

    visionSubsystem = new VisionSubsystem();

    // initialize mock cameras
    mockFrontCamera = mock(PhotonCamera.class);
    when(mockFrontCamera.getName()).thenReturn("frontCamera");
    mockFrontVisionSource =
        visionSubsystem.new VisionSource(mockFrontCamera, Vision.FrontCam.ROBOT_TO_CAM);
    mockBackCamera = mock(PhotonCamera.class);
    when(mockBackCamera.getName()).thenReturn("backCamera");
    mockBackVisionSource =
        visionSubsystem.new VisionSource(mockBackCamera, Vision.BackCam.ROBOT_TO_CAM);
    mockLeftCamera = mock(PhotonCamera.class);
    when(mockLeftCamera.getName()).thenReturn("leftCamera");
    mockLeftVisionSource =
        visionSubsystem
        .new VisionSource(
            mockLeftCamera,
            new Transform3d(
                new Translation3d(0, 0.301, 0.3048), new Rotation3d(0, 0, Math.PI / 2)));

    // inject mock cameras into vision subsystem
    try {
      Field visionSourcesField = visionSubsystem.getClass().getDeclaredField("visionSources");
      visionSourcesField.setAccessible(true);
      visionSourcesField.set(
          visionSubsystem,
          List.of(mockFrontVisionSource, mockBackVisionSource, mockLeftVisionSource));

    } catch (NoSuchFieldException | IllegalAccessException e) {
      e.printStackTrace();
    }
  }

  public static Stream<Arguments> getRobotAngleToPointClosestCameraAtTargetAngleProvider() {
    return Stream.of(
        Arguments.of(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        Arguments.of(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        Arguments.of(Rotation2d.fromDegrees(23), Rotation2d.fromDegrees(23)),
        Arguments.of(Rotation2d.fromDegrees(168), Rotation2d.fromDegrees(-12)),
        Arguments.of(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
        Arguments.of(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0)),
        Arguments.of(Rotation2d.fromDegrees(85), Rotation2d.fromDegrees(-5)),

        // tests for when the robot needs to choose between front and left camera
        Arguments.of(Rotation2d.fromDegrees(44), Rotation2d.fromDegrees(44)),
        Arguments.of(Rotation2d.fromDegrees(46), Rotation2d.fromDegrees(-44))

        // comment to hold the end parenthesis
        );
  }

  @RobotParamTest
  @MethodSource("getRobotAngleToPointClosestCameraAtTargetAngleProvider")
  public void getRobotAngleToPointClosestCameraAtTargetAngle(
      Rotation2d robotAngle, Rotation2d expectedClosestAngle) {
    when(mockFrontCamera.isConnected()).thenReturn(true);
    when(mockBackCamera.isConnected()).thenReturn(true);
    when(mockLeftCamera.isConnected()).thenReturn(true);

    assertEquals(
        Optional.of(expectedClosestAngle),
        visionSubsystem.getRobotAngleToPointClosestCameraAtTargetAngle(robotAngle),
        String.format("Robot angle: %s", robotAngle));
  }

  @RobotTest
  public void getRobotAngleToPointClosestCameraAtTargetAngleWhenNoSources() {
    when(mockFrontCamera.isConnected()).thenReturn(false);
    when(mockBackCamera.isConnected()).thenReturn(false);
    assertEquals(
        Optional.empty(),
        visionSubsystem.getRobotAngleToPointClosestCameraAtTargetAngle(new Rotation2d()));
  }

  @RobotTest
  public void getRobotAngleToPointClosestCameraAtTargetAngleWhenBadSourceConnected() {
    when(mockFrontCamera.isConnected()).thenReturn(true);
    when(mockBackCamera.isConnected()).thenReturn(false);
    when(mockLeftCamera.isConnected()).thenReturn(true);
    assertEquals(
        Optional.of(Rotation2d.fromDegrees(90)),
        visionSubsystem.getRobotAngleToPointClosestCameraAtTargetAngle(
            Rotation2d.fromDegrees(180)));
  }
}
