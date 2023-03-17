package frc.robot.autonomous;

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

  // public static void main (String[] args) {
  //   System.out.println(mirrorSinglePath("auto test"));
  // }

  public static JSONObject mirrorSinglePath(String autoName) {
    
    Path path = Paths.get("src/main/deploy/pathplanner/" + autoName + ".path");

    // Read JSON file
    String fileContents = "";

    try {
      // Files.readAllLines(path).toString
      for (String line : Files.readAllLines(path)) {
        fileContents += line;
      }
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // Converting file into a JSON object
    JSONObject parsedJSON = new JSONObject();
    try {
      parsedJSON = (JSONObject) (new JSONParser().parse(fileContents));
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    JSONArray wayPoints = (JSONArray) parsedJSON.get("waypoints");

    wayPoints.forEach((point) -> mirrorPathPoint(path, point));
    parsedJSON.put("waypoints", wayPoints);

    //writeToFile(path, parsedJSON);
    return parsedJSON;
  }

  public static void mirrorPathPlannerFolder() throws IOException, ParseException {
    eraseMirroredFiles();

    List<Path> listOfPaths = new ArrayList<Path>();
    try (Stream<Path> paths = Files.walk(Paths.get("src/main/deploy/pathplanner"))) {
      paths.filter(Files::isRegularFile).forEach((Path path) -> listOfPaths.add(path));
      for (Path path : listOfPaths) {

        String fileContents = "";

        for (String line : Files.readAllLines(path)) {
          fileContents += line;
        }

        // Read JSON file
        JSONObject parsedJSON = (JSONObject) (new JSONParser().parse(fileContents));

        JSONArray wayPoints = (JSONArray) parsedJSON.get("waypoints");

        wayPoints.forEach((point) -> mirrorPathPoint(path, point));
        parsedJSON.put("waypoints", wayPoints);

        writeToFile(path, parsedJSON);
      }
    }
  }

  public static void writeToFile(Path path, JSONObject parsedJSON) {
    String fileName = path.getFileName().toString().replaceAll(".path", "");
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

  public static void eraseMirroredFiles() throws IOException {
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

    //System.out.println(objectPoint.get("holonomicAngle").getClass().getSimpleName());

    double holonomicAngle =
        objectPoint.get("holonomicAngle") instanceof Long
            ? (double) ((long) objectPoint.get("holonomicAngle"))
            : (double) objectPoint.get("holonomicAngle");

    objectPoint.put("holonomicAngle", (180d - (holonomicAngle % 360)));
  }
}
