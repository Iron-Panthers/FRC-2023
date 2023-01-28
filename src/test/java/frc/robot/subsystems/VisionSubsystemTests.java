package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.geometry.Rotation2d;
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
    mockFrontVisionSource =
        visionSubsystem.new VisionSource(mockFrontCamera, Vision.FrontCam.ROBOT_TO_CAM);
    mockBackCamera = mock(PhotonCamera.class);
    mockBackVisionSource =
        visionSubsystem.new VisionSource(mockBackCamera, Vision.BackCam.ROBOT_TO_CAM);

    // inject mock cameras into vision subsystem
    try {
      Field visionSourcesField = visionSubsystem.getClass().getDeclaredField("visionSources");
      visionSourcesField.setAccessible(true);
      visionSourcesField.set(visionSubsystem, List.of(mockFrontVisionSource, mockBackVisionSource));

    } catch (NoSuchFieldException | IllegalAccessException e) {
      e.printStackTrace();
    }
  }

  @RobotTest
  public void camerasAreConnected() {
    when(mockFrontCamera.isConnected()).thenReturn(true);
    when(mockBackCamera.isConnected()).thenReturn(true);

    assertEquals(true, mockFrontCamera.isConnected());
    assertEquals(true, mockBackCamera.isConnected());
  }

  public static Stream<Arguments> getSourceAngleClosestToRobotAngleProvider() {
    return Stream.of(
        Arguments.of(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        Arguments.of(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(0)),
        Arguments.of(Rotation2d.fromDegrees(350), Rotation2d.fromDegrees(180)));
  }

  @RobotParamTest
  @MethodSource("getSourceAngleClosestToRobotAngleProvider")
  public void getSourceAngleClosestToRobotAngle(
      Rotation2d robotAngle, Rotation2d expectedClosestAngle) {
    when(mockFrontCamera.isConnected()).thenReturn(true);
    when(mockBackCamera.isConnected()).thenReturn(true);

    assertEquals(
        Optional.of(expectedClosestAngle),
        visionSubsystem.getSourceAngleClosestToRobotAngle(robotAngle));
  }

  @RobotTest
  public void getSourceAngleClosestToRobotAngleWhenNoSources() {
    when(mockFrontCamera.isConnected()).thenReturn(false);
    when(mockBackCamera.isConnected()).thenReturn(false);
    assertEquals(
        Optional.empty(), visionSubsystem.getSourceAngleClosestToRobotAngle(new Rotation2d()));
  }
}
