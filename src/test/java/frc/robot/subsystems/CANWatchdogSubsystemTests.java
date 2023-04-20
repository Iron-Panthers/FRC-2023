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
    "BusUtilPerc": -1.0,
    "DeviceArray": [
        {
            "BootloaderRev": "0.5",
            "CANbus": "rio",
            "CANivoreDevName": "",
            "CurrentVers": "22.1.1.0 (Phoenix 5)",
            "HardwareRev": "2.2",
            "ID": 1,
            "IsPROApplication": false,
            "IsPROLicensed": false,
            "LicenseResponseCode": "OK",
            "Licenses": [],
            "LicensesValid": false,
            "ManDate": "Aug 17, 2020",
            "Model": "Talon FX",
            "Name": "Module1 Steer (Device ID 3)",
            "SerialNo": "000E0B500C776800000B0000E60000D8",
            "SoftStatus": "Running Application.",
            "SupportsConfigs": true,
            "SupportsControl_v2": false,
            "SupportsDecoratedSelfTest": false,
            "SupportsLicensing": false,
            "Vendor": "Cross The Road Electronics"
        },
        {
            "BootloaderRev": "0.5",
            "CANbus": "rio",
            "CANivoreDevName": "",
            "CurrentVers": "22.1.1.0 (Phoenix 5)",
            "HardwareRev": "2.2",
            "ID": 2,
            "IsPROApplication": false,
            "IsPROLicensed": false,
            "LicenseResponseCode": "OK",
            "Licenses": [],
            "LicensesValid": false,
            "ManDate": "Oct 1, 2019",
            "Model": "Talon FX",
            "Name": "Module 1 Drive",
            "SerialNo": "000E07B60975E00000140000FA00011D",
            "SoftStatus": "Running Application.",
            "SupportsConfigs": true,
            "SupportsControl_v2": false,
            "SupportsDecoratedSelfTest": false,
            "SupportsLicensing": false,
            "Vendor": "Cross The Road Electronics"
        },
        {
            "BootloaderRev": "0.5",
            "CANbus": "rio",
            "CANivoreDevName": "",
            "CurrentVers": "22.1.1.0 (Phoenix 5)",
            "HardwareRev": "2.2",
            "ID": 3,
            "IsPROApplication": false,
            "IsPROLicensed": false,
            "LicenseResponseCode": "OK",
            "Licenses": [],
            "LicensesValid": false,
            "ManDate": "Sept 17, 2019",
            "Model": "Talon FX",
            "Name": "DriveMotorModule2 (Device ID 10)",
            "SerialNo": "000E07B60975E0000014000105000118",
            "SoftStatus": "Running Application.",
            "SupportsConfigs": true,
            "SupportsControl_v2": false,
            "SupportsDecoratedSelfTest": false,
            "SupportsLicensing": false,
            "Vendor": "Cross The Road Electronics"
        }
    ],
    "GeneralReturn": {
        "Action": "getdevices",
        "CANbus": "",
        "Error": 0,
        "ErrorMessage": "OK",
        "ID": 0,
        "Model": ""
    }
}
        """;
    var ids = CANWatchdogSubsystem.getIds(json);
    assertEquals(List.of(1, 2, 3), ids.toList());
  }

  @RobotTest
  public void jsonDeserializationProducesEmptyStreamOnBadData() {
    var ids = CANWatchdogSubsystem.getIds("This is not JSON");
    assertEquals(List.of(), ids.toList());
  }
}
