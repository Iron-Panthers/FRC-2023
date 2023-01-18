// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.autonomous.commands.AutoTestSequence;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
import frc.robot.commands.IntakeManualCommand;
import frc.robot.commands.PlaceCommand;
import frc.robot.commands.RotateVectorDriveCommand;
import frc.robot.commands.RotateVelocityDriveCommand;
import frc.robot.commands.VibrateControllerCommand;
import frc.robot.subsystems.DataLogger;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.util.ControllerUtil;
import frc.util.Layer;
import frc.util.MacUtil;
import frc.util.Util;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem();
  final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final DataLogger dataLogger = new DataLogger();

  /** controller 1 */
  private final XboxController jason = new XboxController(1);
  /** controller 1 climb layer */
  private final Layer jasonLayer = new Layer(jason::getRightBumper);
  /** controller 0 */
  private final XboxController will = new XboxController(0);

  /** the sendable chooser to select which auto to run. */
  private final SendableChooser<Command> autoSelector = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivebaseSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivebaseSubsystem,
            () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
            () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
            will::getRightBumper));

    SmartDashboard.putBoolean("is comp bot", MacUtil.IS_COMP_BOT);

    // Configure the button bindings
    configureButtonBindings();

    // Create and put autonomous selector to dashboard
    setupAutonomousCommands();
  }

  public void containerTeleopInit() {
    // runs when teleop happens
    CommandScheduler.getInstance().schedule(new VibrateControllerCommand(jason, 5, .5));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // vibrate jason controller when in layer
    jasonLayer.whenChanged(
        (enabled) -> {
          final double power = enabled ? .1 : 0;
          jason.setRumble(RumbleType.kLeftRumble, power);
          jason.setRumble(RumbleType.kRightRumble, power);
        });

    new Button(will::getStartButton)
        .whenPressed(new InstantCommand(drivebaseSubsystem::zeroGyroscope, drivebaseSubsystem));
    new Button(will::getLeftBumper).whenHeld(new DefenseModeCommand(drivebaseSubsystem));

    new Button(will::getLeftStickButton)
        .whenPressed(new HaltDriveCommandsCommand(drivebaseSubsystem));

    DoubleSupplier rotation =
        exponential(
            () ->
                ControllerUtil.deadband(
                    (will.getRightTriggerAxis() + -will.getLeftTriggerAxis()), .1),
            2);
    DoubleSupplier rotationVelocity =
        () ->
            rotation.getAsDouble()
                * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                *
                /** percent of fraction power */
                (will.getAButton() ? .3 : .8);

    new Button(() -> Math.abs(rotation.getAsDouble()) > 0)
        .whenHeld(
            new RotateVelocityDriveCommand(
                drivebaseSubsystem,
                /* drive joystick "y" is passed to x because controller is inverted */
                () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                rotationVelocity,
                will::getRightBumper));

    new Button(
            () ->
                Util.vectorMagnitude(will.getRightY(), will.getRightX())
                    > Drive.ROTATE_VECTOR_MAGNITUDE)
        .whenPressed(
            new RotateVectorDriveCommand(
                drivebaseSubsystem,
                () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                will::getRightY,
                will::getRightX,
                will::getRightBumper));

    new Button(will::getAButton)
        .whileHeld(
            new IntakeManualCommand(
                intakeSubsystem, Constants.Intake.intakePower, -Constants.Intake.ejectPower));

    new Button(will::getYButton)
        .whileHeld(
            new PlaceCommand(
                intakeSubsystem, Constants.Intake.outtakePower, -Constants.Intake.ejectPower));
  }

  /**
   * Adds all autonomous routines to the autoSelector, and places the autoSelector on Shuffleboard.
   */
  private void setupAutonomousCommands() {
    Shuffleboard.getTab("DriverView")
        .addString("NOTES", () -> "Onside is right side. We got this Danny!");

    autoSelector.setDefaultOption(
        "[NEW] AutoTest",
        new AutoTestSequence(
            2, // m/s
            1, // m/s2
            drivebaseSubsystem.getKinematics(),
            drivebaseSubsystem));

    Shuffleboard.getTab("DriverView")
        .add("auto selector", autoSelector)
        .withSize(4, 1)
        .withPosition(6, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new WaitCommand(
    //         /** auto start delay */
    //         7)
    //     .andThen(autoSelector.getSelected());
    return autoSelector.getSelected();
  }

  /**
   * applies deadband and squares axis
   *
   * @param value the axis value to be modified
   * @return the modified axis values
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = ControllerUtil.deadband(value, 0.07);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static DoubleSupplier exponential(DoubleSupplier supplier, double exponential) {
    return () -> {
      double val = supplier.getAsDouble();
      return Math.copySign(Math.pow(val, exponential), val);
    };
  }
}
