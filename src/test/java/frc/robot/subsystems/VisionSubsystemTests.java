package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.RobotParamTest;
import java.lang.reflect.Field;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.photonvision.PhotonCamera;

@TestInstance(Lifecycle.PER_CLASS)
public class VisionSubsystemTests {

  @Mock private PhotonCamera mockFrontCamera;
  @Mock private PhotonCamera mockBackCamera;

  @InjectMocks VisionSubsystem visionSubsystem = new VisionSubsystem();

  @BeforeAll
  public void setup() {
    // set useShuffleboard to false using reflection so that we don't try to use shuffleboard
    // in unit tests
    try {
      Field useShuffleboardField = visionSubsystem.getClass().getDeclaredField("useShuffleboard");
      useShuffleboardField.setAccessible(true);
      useShuffleboardField.set(visionSubsystem, false);

    } catch (NoSuchFieldException | IllegalAccessException e) {
      e.printStackTrace();
    }
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
    assertEquals(
        Optional.of(expectedClosestAngle),
        visionSubsystem.getSourceAngleClosestToRobotAngle(robotAngle));
  }
}
