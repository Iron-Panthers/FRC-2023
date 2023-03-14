// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANWatchdog;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.stream.Stream;

public class CANWatchdogSubsystem extends SubsystemBase {
  private static final String REQUEST = "http://localhost:1250/?action=getdevices";

  private Thread canWatchdogThread;
  private RGBSubsystem rgbSubsystem;

  /**
   * Sleep that handles interrupts and uses an int. Don't do this please?
   *
   * @param millis
   */
  private static void sleep(final int millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      // restore the interrupted status
      Thread.currentThread().interrupt();
    }
  }

  private static final ObjectReader reader =
      new ObjectMapper()
          .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
          .reader()
          .withRootName("DeviceArray");

  protected static Stream<Integer> getIds(String jsonBody) {
    /*
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
        */
    try {
      return reader.readTree(jsonBody).findValues("UniqID").stream()
          .map(JsonNode::numberValue)
          .map(Number::intValue);
    } catch (IOException e) {
      return Stream.empty();
    }
  }

  private static void threadFn() {
    final URI uri = URI.create(REQUEST);
    final HttpClient client = HttpClient.newHttpClient();
    try {
      while (!Thread.currentThread().isInterrupted()) {
        sleep(CANWatchdog.SCAN_DELAY_MS);
        // open a tcp socket to phoenix tuner to get the list of devices
        var body =
            client
                .send(
                    HttpRequest.newBuilder().uri(uri).GET().build(),
                    HttpResponse.BodyHandlers.ofString())
                .body();

        if (body == null) continue;
      }
    } catch (InterruptedException e) {
      // restore the interrupted status
      Thread.currentThread().interrupt();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /** Creates a new CANWatchdog. */
  public CANWatchdogSubsystem() {
    canWatchdogThread = new Thread(CANWatchdogSubsystem::threadFn, "CAN Watchdog Thread");
    canWatchdogThread.setDaemon(true);
    canWatchdogThread.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
