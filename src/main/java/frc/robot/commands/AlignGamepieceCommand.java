// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.util.pathing.RubenManueverGenerator;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;



public class AlignGamepieceCommand extends SequentialCommandGroup {

  DriveToPlaceCommand driveToPlace;

    /** Creates a new DriveToPlaceCommand. */
   public AlignGamepieceCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      RubenManueverGenerator manueverGenerator,
      Supplier<Pose2d> observationPose,
      Supplier<Pose2d> finalPose,
      double observationTime,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      BooleanSupplier isRobotRelativeRelativeSupplier,
      Optional<RGBSubsystem> rgbSubsystem,
      Optional<GenericHID> failureRumbleDevice) {
    // Use addRequirements() here to declare subsystem dependencies.


    addCommands(
      new DriveToPlaceCommand(
        drivebaseSubsystem, 
        manueverGenerator, 
        observationPose, 
        finalPose, 
        observationTime, 
        translationXSupplier,
        () -> {

          var alliance = DriverStation.getAlliance();

          return switch (alliance) {
            case Blue -> translationYSupplier.getAsDouble() - drivebaseSubsystem.getSensorDistance();
            case Red -> translationYSupplier.getAsDouble() + drivebaseSubsystem.getSensorDistance();
            case Invalid -> {
              System.out.println("Invalid alliance, defaulting to blue align");
              yield  translationYSupplier.getAsDouble() - drivebaseSubsystem.getSensorDistance();
            }
            default -> {
              System.out.printf("Unknown alliance %s, defaulting to blue align", alliance);
              yield  translationYSupplier.getAsDouble() - drivebaseSubsystem.getSensorDistance();
            }
          };
        },       
        isRobotRelativeRelativeSupplier,
        rgbSubsystem,
        failureRumbleDevice)
    );

  }


  
  /**
   * Creates a new DriveToPlaceCommand without an observation pose.
   *
   * @param drivebaseSubsystem The drivebase subsystem.
   * @param visionSubsystem The vision subsystem.
   * @param finalPose The final pose to put the robot in.
   */
  public AlignGamepieceCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      RubenManueverGenerator manueverGenerator,
      Supplier<Pose2d> finalPose,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      BooleanSupplier isRobotRelativeRelativeSupplier,
      Optional<RGBSubsystem> rgbSubsystem,
      Optional<GenericHID> failureRumbleDevice) {
    this(
        drivebaseSubsystem,
        manueverGenerator,
        finalPose,
        finalPose,
        0.1,
        translationXSupplier,
        translationYSupplier,
        isRobotRelativeRelativeSupplier,
        rgbSubsystem,
        failureRumbleDevice);
  }
}
