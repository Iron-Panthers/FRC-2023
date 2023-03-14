package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.RobotTest;
import java.util.List;

public class CANWatchdogSubsystemTests {

  @RobotTest
  public void jsonDeserializes() {
    String json =
        """
{
	"DeviceArray": [{
			"BootloaderRev": "0.2",
			"CurrentVers": "4.1",
			"DynID": 17102859,
			"HardwareRev": "1.0",
			"ID": 17102859,
			"ManDate": "Nov 19, 2017",
			"Model": "Victor SPX",
			"Name": "Victor SPX 1",
			"SoftStatus": "Running Application",
			"UniqID": 5,
			"Vendor": "VEX Robotics"
		},
		{
			"BootloaderRev": "2.6",
			"CurrentVers": "4.1",
			"DynID": 33880073,
			"HardwareRev": "1.4",
			"ID": 33880073,
			"ManDate": "Nov 3, 2017",
			"Model": "Talon SRX",
			"Name": "Talon SRX 1",
			"SoftStatus": "Running Application",
			"UniqID": 4,
			"Vendor": "Cross The Road Electronics"
		}
	]
}
        """;
    var ids = CANWatchdogSubsystem.getIds(json);
    assertEquals(List.of(5, 4), ids.toList());
  }

  @RobotTest
  public void jsonDeserializationProducesEmptyStreamOnBadData() {
    var ids = CANWatchdogSubsystem.getIds("This is not JSON");
    assertEquals(List.of(), ids.toList());
  }
}
