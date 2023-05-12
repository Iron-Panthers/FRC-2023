package frc.util.pathing;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.json.simple.*;
import org.json.simple.parser.*;

public class MirrorPath {

  // For testing purposes
  // public static void main(String[] args) {
  //   eraseMirroredFiles();
  //   JSONObject redPath = mirrorSinglePath("n9 1cone + mobility engage");
  //   System.out.println(redPath);
  //   writeToFile("redTest", redPath);
  // }

  public static JSONObject mirrorSinglePath(String autoName) {

    Path path = Paths.get("src/main/deploy/pathplanner/" + autoName + ".path");

    String fileContents = "";
    try {
      fileContents = String.join(",", Files.readAllLines(path));
      // Converting file into a JSON object
      JSONObject parsedJSON = (JSONObject) (new JSONParser().parse(fileContents));

      // Getting the waypoints
      JSONArray wayPoints = (JSONArray) parsedJSON.get("waypoints");

      // Mirroring each waypoint
      wayPoints.forEach((point) -> mirrorPathPoint(path, point));

      // Overriding the original waypoints
      parsedJSON.put("waypoints", wayPoints);

      return parsedJSON;
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    return new JSONObject();
  }

  public static void mirrorPathPlannerFolder() {
    eraseMirroredFiles();

    // List of all the paths
    List<Path> listOfPaths = new ArrayList<Path>();

    // Reading all the paths of all the auto files from the pathplanner folder
    try (Stream<Path> paths = Files.walk(Paths.get("src/main/deploy/pathplanner"))) {
      paths.filter(Files::isRegularFile).forEach((Path path) -> listOfPaths.add(path));
      for (Path path : listOfPaths) {

        String fileContents = String.join(",", Files.readAllLines(path));

        // Read JSON file
        JSONObject parsedJSON = (JSONObject) (new JSONParser().parse(fileContents));

        JSONArray wayPoints = (JSONArray) parsedJSON.get("waypoints");

        wayPoints.forEach((point) -> mirrorPathPoint(path, point));
        parsedJSON.put("waypoints", wayPoints);

        writeToFile(path, parsedJSON);
      }
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  public static void writeToFile(Path path, JSONObject parsedJSON) {
    String fileName = path.getFileName().toString().replaceAll(".path", "");
    writeToFile(fileName, parsedJSON);
  }

  public static void writeToFile(String fileName, JSONObject parsedJSON) {
    try {
      FileWriter mirroredPathFile =
          new FileWriter("src/main/deploy/pathplanner/" + fileName + ".mirror.path");
      mirroredPathFile.write(parsedJSON.toJSONString());
      mirroredPathFile.close();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  public static void eraseMirroredFiles() {
    try (Stream<Path> paths = Files.walk(Paths.get("src/main/deploy/pathplanner"))) {
      paths
          .filter((Path path) -> path.toString().contains(".mirror.path"))
          .forEach(
              (Path path) -> {
                try {
                  Files.deleteIfExists(path);
                } catch (IOException e) {
                  // TODO Auto-generated catch block
                  e.printStackTrace();
                }
              });
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public static void mirrorPathPoint(Path path, Object point) {
    final double FIELD_LENGTH = 16.54175;
    final double FIELD_HEIGHT = 8.0137;
    JSONObject objectPoint = (JSONObject) point;
    JSONObject anchorPoint = (JSONObject) objectPoint.get("anchorPoint");

    if (!anchorPoint.isEmpty()) {
      anchorPoint.put("x", FIELD_LENGTH - ((Number) anchorPoint.get("x")).doubleValue());
      anchorPoint.put("y", FIELD_HEIGHT - ((Number) anchorPoint.get("y")).doubleValue());
    }

    System.out.println(objectPoint.get("prevControl"));

    if (objectPoint.get("prevControl") != null) {

      JSONObject prevPoint = (JSONObject) objectPoint.get("prevControl");
      
      if (!prevPoint.isEmpty()) {
        prevPoint.put("x", FIELD_LENGTH - ((Number) prevPoint.get("x")).doubleValue());
        prevPoint.put("y", FIELD_HEIGHT - ((Number) prevPoint.get("y")).doubleValue());
      }
    }

    if (objectPoint.get("nextControl") != null) {

      JSONObject nextControl = (JSONObject) objectPoint.get("nextControl");

      if (!nextControl.isEmpty()) {
        nextControl.put("x", FIELD_LENGTH - ((Number) nextControl.get("x")).doubleValue());
        nextControl.put("y", FIELD_LENGTH - ((Number) nextControl.get("y")).doubleValue());

      }
    }

    double holonomicAngle = ((Number) objectPoint.get("holonomicAngle")).doubleValue();

    objectPoint.put("holonomicAngle", ((holonomicAngle) + 180) % 360);
  }
}
