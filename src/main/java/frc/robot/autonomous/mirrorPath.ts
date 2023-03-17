// read json file at the arg path deno

const path = Deno.args[0];
console.log(path);

const mirrorPath = async (path: string) => {
  const file = await Deno.readTextFile(path);

  const json = JSON.parse(file);

  const waypoints = [];

  /**
 * {
  anchorPoint: { x: 5.854098574753403, y: 4.191433119491278 },
  prevControl: { x: 5.854098574753403, y: 5.009781789511119 },
  nextControl: { x: 5.854098574753403, y: 3.166884066258022 },
  holonomicAngle: 180,
  isReversal: false,
  velOverride: null,
  isLocked: false,
  isStopPoint: false,
  stopEvent: { names: [], executionBehavior: "parallel", waitBehavior: "none", waitTime: 0 }
}
 */
  type Waypoint = {
    anchorPoint: { x: number; y: number };
    prevControl: { x: number; y: number } | null;
    nextControl: { x: number; y: number } | null;
    holonomicAngle: number;
    isReversal: boolean;
    velOverride: null | number;
    isLocked: boolean;
    isStopPoint: boolean;
    stopEvent: {
      names: string[];
      executionBehavior: "parallel" | "sequential";
      waitBehavior: "none" | "untilDone" | "untilTime";
      waitTime: number;
    };
  };

  const FIELD_LENGTH = 16.54175;
  for (const point of json.waypoints as Waypoint[]) {
    /**
  public static Pose2d findRedPose(Pose2d bluePose) {
    return new Pose2d(
        new Translation2d(
            FieldObstructionMap.FIELD_LENGTH - bluePose.getTranslation().getX(),
            bluePose.getTranslation().getY()),
        mirrorRotation.minus(bluePose.getRotation()));
   */
    const newPoint: Waypoint = {
      anchorPoint: {
        x: FIELD_LENGTH - point.anchorPoint.x,
        y: point.anchorPoint.y,
      },
      prevControl: point.prevControl
        ? { x: FIELD_LENGTH - point.prevControl.x, y: point.prevControl.y }
        : null,
      nextControl: point.nextControl
        ? { x: FIELD_LENGTH - point.nextControl.x, y: point.nextControl.y }
        : null,
      holonomicAngle: (180 - point.holonomicAngle) % 360,
      isReversal: point.isReversal,
      velOverride: point.velOverride,
      isLocked: point.isLocked,
      isStopPoint: point.isStopPoint,
      stopEvent: point.stopEvent,
    };

    waypoints.push(newPoint);
  }

  const newJson = {
    ...json,
    waypoints,
  };

  // write the new json file to [path].mirror.path
  await Deno.writeFile(
    `${path}.mirror.path`,
    new TextEncoder().encode(JSON.stringify(newJson, null, 2))
  );
};

// iterate over all the paths in the paths folder
for await (const path of Deno.readDir(
  Deno.cwd() + "/src/main/deploy/pathplanner/"
)) {
  if (path.name.endsWith(".mirror.path")) {
    // delete the mirror path
    await Deno.remove(Deno.cwd() + "/src/main/deploy/pathplanner/" + path.name);
  }
}

for await (const path of Deno.readDir(
  Deno.cwd() + "/src/main/deploy/pathplanner/"
)) {
  if (path.name.endsWith(".path")) {
    await mirrorPath(Deno.cwd() + "/src/main/deploy/pathplanner/" + path.name);
  }
}
