package frc.util.pathing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

public class LoadMirrorPath {
  private LoadMirrorPath() {}

  public static Supplier<PathPlannerTrajectory> loadPath(
      String pathName, double maxVelocityMetersPerSecond, double maxAccelerationMetersPerSecondSq) {
    PathPlannerTrajectory bluePath =
        PathPlanner.loadPath(
            pathName, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    PathPlannerTrajectory redPath =
        PathPlannerRed.loadRedPath(
            pathName, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    return () -> DriverStation.getAlliance() == Alliance.Red ? redPath : bluePath;
  }

  public static List<Supplier<PathPlannerTrajectory>> loadPathGroup(
      String pathName, double maxVelocityMetersPerSecond, double maxAccelerationMetersPerSecondSq) {
    List<PathPlannerTrajectory> bluePath =
        PathPlanner.loadPathGroup(
            pathName, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    List<PathPlannerTrajectory> redPath =
        PathPlannerRed.loadRedPathGroup(
            pathName, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    return IntStream.range(0, bluePath.size())
        .mapToObj(
            i ->
                (Supplier<PathPlannerTrajectory>)
                    () ->
                        DriverStation.getAlliance() == Alliance.Red
                            ? redPath.get(i)
                            : bluePath.get(i))
        .collect(Collectors.toList());
  }

  // Don't know if we should make it extend Path planner, not rly necessary
  private class PathPlannerRed extends PathPlanner {

    // Load mirrored path group

    public static List<PathPlannerTrajectory> loadRedPathGroup(
        String name, double maxVel, double maxAccel) {
      return loadRedPathGroup(name, false, new PathConstraints(maxVel, maxAccel));
    }

    /**
     * Load a path file from storage as a path group. This will separate the path into multiple
     * paths based on the waypoints marked as "stop points"
     *
     * @param name The name of the path group to load
     * @param reversed Should the robot follow this path group reversed
     * @param constraint The PathConstraints (max velocity, max acceleration) of the first path in
     *     the group
     * @param constraints The PathConstraints (max velocity, max acceleration) of the remaining
     *     paths in the group. If there are less constraints than paths, the last constrain given
     *     will be used for the remaining paths.
     * @return A List of all generated paths in the group
     */
    public static List<PathPlannerTrajectory> loadRedPathGroup(
        String name, boolean reversed, PathConstraints constraint, PathConstraints... constraints) {
      List<PathConstraints> allConstraints = new ArrayList<>();
      allConstraints.add(constraint);
      allConstraints.addAll(Arrays.asList(constraints));

      // Get the mirrored path
      JSONObject json = MirrorPath.mirrorSinglePath(name);

      List<Waypoint> waypoints = getWaypointsFromJson(json);
      List<EventMarker> markers = getMarkersFromJson(json);

      List<List<Waypoint>> splitWaypoints = new ArrayList<>();
      List<List<EventMarker>> splitMarkers = new ArrayList<>();

      List<Waypoint> currentPath = new ArrayList<>();
      for (int i = 0; i < waypoints.size(); i++) {
        Waypoint w = waypoints.get(i);

        currentPath.add(w);
        if (w.isStopPoint || i == waypoints.size() - 1) {
          // Get the markers that should be part of this path and correct their positions
          List<EventMarker> currentMarkers = new ArrayList<>();
          for (EventMarker marker : markers) {
            if (marker.waypointRelativePos >= waypoints.indexOf(currentPath.get(0))
                && marker.waypointRelativePos <= i) {
              currentMarkers.add(
                  new EventMarker(
                      marker.names,
                      marker.waypointRelativePos - waypoints.indexOf(currentPath.get(0))));
            }
          }
          splitMarkers.add(currentMarkers);

          splitWaypoints.add(currentPath);
          currentPath = new ArrayList<>();
          currentPath.add(w);
        }
      }

      if (splitWaypoints.size() != splitMarkers.size()) {
        throw new RuntimeException(
            "Size of splitWaypoints does not match splitMarkers. Something went very wrong");
      }

      List<PathPlannerTrajectory> pathGroup = new ArrayList<>();
      boolean shouldReverse = reversed;
      for (int i = 0; i < splitWaypoints.size(); i++) {
        PathConstraints currentConstraints;
        if (i > allConstraints.size() - 1) {
          currentConstraints = allConstraints.get(allConstraints.size() - 1);
        } else {
          currentConstraints = allConstraints.get(i);
        }

        pathGroup.add(
            new PathPlannerTrajectory(
                splitWaypoints.get(i),
                splitMarkers.get(i),
                currentConstraints,
                shouldReverse,
                true));

        // Loop through waypoints and invert shouldReverse for every reversal point.
        // This makes sure that other paths in the group are properly reversed.
        for (int j = 1; j < splitWaypoints.get(i).size(); j++) {
          if (splitWaypoints.get(i).get(j).isReversal) {
            shouldReverse = !shouldReverse;
          }
        }
      }

      return pathGroup;
    }

    // Load a single mirrored path

    /**
     * Load a mirrored path file
     *
     * @param name The name of the path to load
     * @param maxVel Max velocity of the path
     * @param maxAccel Max velocity of the path
     * @return The generated path
     */
    public static PathPlannerTrajectory loadRedPath(String name, double maxVel, double maxAccel) {
      return loadRedPath(name, new PathConstraints(maxVel, maxAccel), false);
    }

    /**
     * Load a mirrored path file
     *
     * @param name The name of the path to load
     * @param constraints Max velocity and acceleration constraints of the path
     * @param reversed Should the robot follow the path reversed
     * @return The generated path
     */
    public static PathPlannerTrajectory loadRedPath(
        String name, PathConstraints constraints, boolean reversed) {
      JSONObject json = MirrorPath.mirrorSinglePath(name);

      List<Waypoint> waypoints = getWaypointsFromJson(json);
      List<EventMarker> markers = getMarkersFromJson(json);

      return new PathPlannerTrajectory(waypoints, markers, constraints, reversed, true);
    }

    // Private methods that PathPlanner locked away, but we need to use

    private static List<Waypoint> getWaypointsFromJson(JSONObject json) {
      JSONArray jsonWaypoints = (JSONArray) json.get("waypoints");

      List<Waypoint> waypoints = new ArrayList<>();

      for (Object waypoint : jsonWaypoints) {
        JSONObject jsonWaypoint = (JSONObject) waypoint;

        JSONObject jsonAnchor = (JSONObject) jsonWaypoint.get("anchorPoint");
        Translation2d anchorPoint =
            new Translation2d(
                ((Number) jsonAnchor.get("x")).doubleValue(),
                ((Number) jsonAnchor.get("y")).doubleValue());

        JSONObject jsonPrevControl = (JSONObject) jsonWaypoint.get("prevControl");
        Translation2d prevControl = null;
        if (jsonPrevControl != null) {
          prevControl =
              new Translation2d(
                  ((Number) jsonPrevControl.get("x")).doubleValue(),
                  ((Number) jsonPrevControl.get("y")).doubleValue());
        }

        JSONObject jsonNextControl = (JSONObject) jsonWaypoint.get("nextControl");
        Translation2d nextControl = null;
        if (jsonNextControl != null) {
          nextControl =
              new Translation2d(
                  ((Number) jsonNextControl.get("x")).doubleValue(),
                  ((Number) jsonNextControl.get("y")).doubleValue());
        }

        Rotation2d holonomicAngle = null;
        if (jsonWaypoint.get("holonomicAngle") != null) {
          holonomicAngle =
              Rotation2d.fromDegrees(((Number) jsonWaypoint.get("holonomicAngle")).doubleValue());
        }
        boolean isReversal = (boolean) jsonWaypoint.get("isReversal");
        Object isStopPointObj = jsonWaypoint.get("isStopPoint");
        boolean isStopPoint = false;
        if (isStopPointObj != null) isStopPoint = (boolean) isStopPointObj;
        double velOverride = -1;
        if (jsonWaypoint.get("velOverride") != null) {
          velOverride = ((Number) jsonWaypoint.get("velOverride")).doubleValue();
        }

        PathPlannerTrajectory.StopEvent stopEvent = new PathPlannerTrajectory.StopEvent();
        if (jsonWaypoint.get("stopEvent") != null) {
          List<String> names = new ArrayList<>();
          PathPlannerTrajectory.StopEvent.ExecutionBehavior executionBehavior =
              PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL;
          PathPlannerTrajectory.StopEvent.WaitBehavior waitBehavior =
              PathPlannerTrajectory.StopEvent.WaitBehavior.NONE;
          double waitTime = 0;

          JSONObject stopEventJson = (JSONObject) jsonWaypoint.get("stopEvent");
          if (stopEventJson.get("names") != null) {
            JSONArray namesArray = (JSONArray) stopEventJson.get("names");
            for (Object name : namesArray) {
              names.add(name.toString());
            }
          }
          if (stopEventJson.get("executionBehavior") != null) {
            PathPlannerTrajectory.StopEvent.ExecutionBehavior behavior =
                PathPlannerTrajectory.StopEvent.ExecutionBehavior.fromValue(
                    stopEventJson.get("executionBehavior").toString());

            if (behavior != null) {
              executionBehavior = behavior;
            }
          }
          if (stopEventJson.get("waitBehavior") != null) {
            PathPlannerTrajectory.StopEvent.WaitBehavior behavior =
                PathPlannerTrajectory.StopEvent.WaitBehavior.fromValue(
                    stopEventJson.get("waitBehavior").toString());

            if (behavior != null) {
              waitBehavior = behavior;
            }
          }
          if (stopEventJson.get("waitTime") != null) {
            waitTime = ((Number) stopEventJson.get("waitTime")).doubleValue();
          }

          stopEvent =
              new PathPlannerTrajectory.StopEvent(names, executionBehavior, waitBehavior, waitTime);
        }

        waypoints.add(
            new Waypoint(
                anchorPoint,
                prevControl,
                nextControl,
                velOverride,
                holonomicAngle,
                isReversal,
                isStopPoint,
                stopEvent));
      }

      return waypoints;
    }

    private static List<EventMarker> getMarkersFromJson(JSONObject json) {
      JSONArray jsonMarkers = (JSONArray) json.get("markers");

      List<EventMarker> markers = new ArrayList<>();

      if (jsonMarkers != null) {
        for (Object marker : jsonMarkers) {
          JSONObject jsonMarker = (JSONObject) marker;

          JSONArray eventNames = (JSONArray) jsonMarker.get("names");
          List<String> names = new ArrayList<>();
          if (eventNames != null) {
            for (Object eventName : eventNames) {
              names.add((String) eventName);
            }
          } else {
            // Handle transition between one-event markers and multi-event markers. Remove next
            // season
            names.add((String) jsonMarker.get("name"));
          }
          markers.add(new EventMarker(names, ((Number) jsonMarker.get("position")).doubleValue()));
        }
      }

      return markers;
    }
  }
}
