package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import frc.RobotTest;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.MockedConstruction;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class RGBSubsystemTests {
  private MockedConstruction<CANdle> mockCandle;
  private CANdle candle;

  private RGBSubsystem rgbSubsystem;

  @BeforeEach
  public void setup() {
    mockCandle = Mockito.mockConstruction(CANdle.class);
    rgbSubsystem = new RGBSubsystem();
    candle = mockCandle.constructed().get(0);
    when(candle.animate(any(Animation.class))).thenReturn(ErrorCode.OK);
  }

  @AfterEach
  public void teardown() {
    mockCandle.close();
  }

  @RobotTest
  public void mocksInjectCorrectly() {
    // verify that the mock was injected correctly
    assertNotNull(candle);
    assertEquals(ErrorCode.OK, candle.animate(new RainbowAnimation()));
  }
}
