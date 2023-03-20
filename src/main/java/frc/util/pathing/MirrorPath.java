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
  //   JSONObject redPath = mirrorSinglePath("n2 engage");
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
    JSONObject objectPoint = (JSONObject) point;
    JSONObject anchorPoint = (JSONObject) objectPoint.get("anchorPoint");

    if (!anchorPoint.isEmpty()) {
      anchorPoint.put("x", FIELD_LENGTH - (double) anchorPoint.get("x"));
    }

    System.out.println(objectPoint.get("prevControl"));

    if (objectPoint.get("prevControl") != null) {

      JSONObject prevPoint = (JSONObject) objectPoint.get("prevControl");
      // System.out.println("Prev control: ");
      // System.out.println((double) prevPoint.get("x"));

      // System.out.println("[Mirrored] Prev control: ");
      // System.out.println(FIELD_LENGTH - (double) prevPoint.get("x"));

      System.out.println(prevPoint.get("x").getClass().getSimpleName());

      if (!prevPoint.isEmpty()) {
        prevPoint.put("x", FIELD_LENGTH - (double) prevPoint.get("x"));
      }
    }

    if (objectPoint.get("nextControl") != null) {

      JSONObject nextControl = (JSONObject) objectPoint.get("nextControl");

      // System.out.println("Next control: ");
      // System.out.println((double) nextControl.get("x"));

      // System.out.println("[Mirrored] Next control: ");
      // System.out.println(FIELD_LENGTH - (double) nextControl.get("x"));

      if (!nextControl.isEmpty()) {
        nextControl.put("x", FIELD_LENGTH - (double) nextControl.get("x"));
      }
    }

    // System.out.println(objectPoint.get("holonomicAngle").getClass().getSimpleName());

    double holonomicAngle =
        objectPoint.get("holonomicAngle") instanceof Long
            ? (double) ((long) objectPoint.get("holonomicAngle"))
            : (double) objectPoint.get("holonomicAngle");

    objectPoint.put("holonomicAngle", (180d - (holonomicAngle % 360)));
  }
}
