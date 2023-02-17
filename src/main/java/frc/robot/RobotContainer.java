// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drive;
import static frc.robot.Constants.Intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Arm;
import frc.robot.autonomous.commands.AutoTestSequence;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.DriveToPlaceCommand;
import frc.robot.commands.ForceOuttakeCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.RotateVectorDriveCommand;
import frc.robot.commands.RotateVelocityDriveCommand;
import frc.robot.commands.StartSpindexerHopperCommand;
import frc.robot.commands.VibrateControllerCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeModes;
import frc.robot.subsystems.NetworkWatchdogSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SpindexerHopperSubsystem;
import frc.robot.subsystems.OuttakeSubsystem.Modes;
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

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final NetworkWatchdogSubsystem networkWatchdogSubsystem = new NetworkWatchdogSubsystem();

  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();

  private final SpindexerHopperSubsystem spindexerHopperSubsystem = new SpindexerHopperSubsystem();

  /** controller 1 */
  private final CommandXboxController jason = new CommandXboxController(1);
  /** controller 1 climb layer */
  private final Layer jasonLayer = new Layer(jason.rightBumper());
  /** controller 0 */
  private final CommandXboxController will = new CommandXboxController(0);

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
            will.rightBumper()));

    armSubsystem.setDefaultCommand(
        new ArmManualCommand(
            armSubsystem,
            () -> ControllerUtil.deadband(-jason.getLeftY(), 0.2),
            () -> ControllerUtil.deadband(jason.getRightY(), 0.2)));

    SmartDashboard.putBoolean("is comp bot", MacUtil.IS_COMP_BOT);

    // Configure the button bindings
    configureButtonBindings();

    // Create and put autonomous selector to dashboard
    setupAutonomousCommands();
  }

  /**
   * Use this method to do things as the drivers gain control of the robot. We use it to vibrate the
   * driver b controller to notice accidental swaps.
   *
   * <p>Please use this very, very sparingly. It doesn't exist by default for good reason.
   */
  public void containerTeleopInit() {
    // runs when teleop happens
    CommandScheduler.getInstance().schedule(new VibrateControllerCommand(jason, 5, .5));
  }

  /**
   * Use this method to do things as soon as the robot starts being used. We use it to stop doing
   * things that could be harmful or undesirable during game play--rebooting the network switch is a
   * good example. Subsystems need to be explicitly wired up to this method.
   *
   * <p>Depending on which mode the robot is enabled in, this will either be called before auto or
   * before teleop, whichever is first.
   *
   * <p>Please use this very, very sparingly. It doesn't exist by default for good reason.
   */
  public void containerMatchStarting() {
    // runs when the match starts
    networkWatchdogSubsystem.matchStarting();
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
          jason.getHID().setRumble(RumbleType.kLeftRumble, power);
          jason.getHID().setRumble(RumbleType.kRightRumble, power);
        });

    jason.a().onTrue(new StartSpindexerHopperCommand(spindexerHopperSubsystem, Modes.IDLE));

    will.start().onTrue(new InstantCommand(drivebaseSubsystem::zeroGyroscope, drivebaseSubsystem));
    will.leftBumper().whileTrue(new DefenseModeCommand(drivebaseSubsystem));

    will.leftStick().onTrue(new HaltDriveCommandsCommand(drivebaseSubsystem));

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
                (will.getHID().getAButton() ? .3 : .8);

    new Trigger(() -> Math.abs(rotation.getAsDouble()) > 0)
        .whileTrue(
            new RotateVelocityDriveCommand(
                drivebaseSubsystem,
                /* drive joystick "y" is passed to x because controller is inverted */
                () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                rotationVelocity,
                will.rightBumper()));

    new Trigger(
            () ->
                Util.vectorMagnitude(will.getRightY(), will.getRightX())
                    > Drive.ROTATE_VECTOR_MAGNITUDE)
        .onTrue(
            new RotateVectorDriveCommand(
                drivebaseSubsystem,
                () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                will::getRightY,
                will::getRightX,
                will.rightBumper()));

    // inline command to generate path on the fly that drives to 5,5 at heading zero
    will.b()
        .onTrue(
            new DriveToPlaceCommand(
                drivebaseSubsystem, new Pose2d(3.5, 2.2, Rotation2d.fromDegrees(0)), .2, .5));

    will.y()
        .onTrue(
            new DriveToPlaceCommand(
                drivebaseSubsystem, new Pose2d(3.2, .5, Rotation2d.fromDegrees(170)), .2, .5));

    jasonLayer.on(jason.x())
        .whileTrue(
            new IntakeCommand(
                intakeSubsystem, IntakeModes.DEPLOY))
        .onFalse(
            new IntakeCommand(
                intakeSubsystem, IntakeModes.RETRACT));

    jasonLayer
        .off(jason.leftTrigger())
        .whileTrue(new ForceOuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE));
    jasonLayer
        .off(jason.rightTrigger())
        .onTrue(new OuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.OUTTAKE));
    jasonLayer
        .off(jason.x())
        .onTrue(new OuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.OFF));
    jasonLayer
        .off(jason.a())
        .onTrue(
            new ArmPositionCommand(
                armSubsystem,
                Arm.Setpoints.GroundIntake.ANGLE,
                Arm.Setpoints.GroundIntake.EXTENSION))
        .whileTrue(new ForceOuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE));
    jasonLayer
        .off(jason.b())
        .onTrue(
            new ArmPositionCommand(
                armSubsystem, Arm.Setpoints.ShelfIntake.ANGLE, Arm.Setpoints.ShelfIntake.EXTENSION))
        .whileTrue(new ForceOuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE));
    jasonLayer
        .off(jason.y())
        .onTrue(
            new ArmPositionCommand(
                armSubsystem,
                Arm.Setpoints.Angles.STARTING_ANGLE,
                Arm.Setpoints.Extensions.MIN_EXTENSION));

    jasonLayer
        .on(jason.a())
        .whileTrue(
            new ArmPositionCommand(
                armSubsystem, Arm.Setpoints.ScoreLow.ANGLE, Arm.Setpoints.ScoreLow.EXTENSION))
        .onFalse(new OuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.OUTTAKE));
    jasonLayer
        .on(jason.b())
        .whileTrue(
            new ArmPositionCommand(
                armSubsystem, Arm.Setpoints.ScoreMid.ANGLE, Arm.Setpoints.ScoreMid.EXTENSION))
        .onFalse(new ArmPositionCommand(
            armSubsystem, Arm.Setpoints.ScoreMid.CAPPED_ANGLE, Arm.Setpoints.ScoreMid.EXTENSION).alongWith(new OuttakeCommand(outtakeSubsystem, Modes.OFF)));
    jasonLayer
        .on(jason.y())
        .whileTrue(
            new ArmPositionCommand(
                armSubsystem, Arm.Setpoints.ScoreHigh.ANGLE, Arm.Setpoints.ScoreHigh.EXTENSION))
        .onFalse(new OuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.OUTTAKE));
  }

  /**
   * Adds all autonomous routines to the autoSelector, and places the autoSelector on Shuffleboard.
   */
  private void setupAutonomousCommands() {
    Shuffleboard.getTab("DriverView")
        .addString("NOTES", () -> "...win?")
        .withSize(3, 1)
        .withPosition(0, 0);

    autoSelector.setDefaultOption(
        "[NEW] AutoTest",
        new AutoTestSequence(
            2, // m/s
            1, // m/s2
            drivebaseSubsystem));

    Shuffleboard.getTab("DriverView")
        .add("auto selector", autoSelector)
        .withSize(4, 1)
        .withPosition(7, 0);
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
