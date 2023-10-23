// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drive;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Config;
import frc.robot.Constants.Drive;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.ForceOuttakeSubsystemModeCommand;
import frc.robot.commands.GroundPickupCommand;
import frc.robot.commands.HashMapCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeShootCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.ScoreStep;
import frc.robot.commands.SetOuttakeModeCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.commands.TestElevatorCommand;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.commands.ZeroIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.CANWatchdogSubsystem;
import frc.robot.subsystems.ElevatorTestSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NetworkWatchdogSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.util.ControllerUtil;
import frc.util.Layer;
import frc.util.MacUtil;
import frc.util.NodeSelectorUtility;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeSelection;
import frc.util.NodeSelectorUtility.NodeType;
import frc.util.SharedReference;
import frc.util.pathing.RubenManueverGenerator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private final ElevatorTestSubsystem elevatorSubsystem = new ElevatorTestSubsystem();
  private final RGBSubsystem rgbSubsystem = new RGBSubsystem();

  private final NetworkWatchdogSubsystem networkWatchdogSubsystem =
      new NetworkWatchdogSubsystem(Optional.of(rgbSubsystem));

  private final CANWatchdogSubsystem canWatchdogSubsystem = new CANWatchdogSubsystem(rgbSubsystem);

  private final RubenManueverGenerator manueverGenerator = new RubenManueverGenerator();

  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem(Optional.of(rgbSubsystem));

  private final SharedReference<NodeSelection> currentNodeSelection =
      new SharedReference<>(new NodeSelection(NodeSelectorUtility.defaultNodeStack, Height.HIGH));

  private final CommandXboxController will = new CommandXboxController(0);

  /** the sendable chooser to select which auto to run. */
  private final SendableChooser<Command> autoSelector = new SendableChooser<>();

  private GenericEntry autoDelay;

  private final ShuffleboardTab driverView = Shuffleboard.getTab("DriverView");


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    SmartDashboard.putBoolean("is comp bot", MacUtil.IS_COMP_BOT);
    SmartDashboard.putBoolean("show debug data", Config.SHOW_SHUFFLEBOARD_DEBUG_DATA);
    SmartDashboard.putBoolean("don't init swerve modules", Config.DISABLE_SWERVE_MODULE_INIT);

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
    canWatchdogSubsystem.matchStarting();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    will.povUp().onTrue(new TestElevatorCommand(elevatorSubsystem, 1));
    will.povRight().onTrue(new TestElevatorCommand(elevatorSubsystem, 0.5));
    will.povDown().onTrue(new TestElevatorCommand(elevatorSubsystem, 0));
  }

  /**
   * Adds all autonomous routines to the autoSelector, and places the autoSelector on Shuffleboard.
   */
  private void setupAutonomousCommands() {
    if (Config.RUN_PATHPLANNER_SERVER) {
      PathPlannerServer.startServer(5811);
    }

    driverView.addString("NOTES", () -> "...win?").withSize(3, 1).withPosition(0, 0);

    final List<ScoreStep> drivingCubeOuttake =
        List.of(
            new ScoreStep(new ArmState(35, Arm.Setpoints.Extensions.MIN_EXTENSION)).canWaitHere(),
            new ScoreStep(OuttakeSubsystem.Modes.OUTTAKE));
    final boolean[] intakeLow = {false};
    final Map<String, Command> eventMap =
        Map.of(
            "stow arm",
            new ArmPositionCommand(armSubsystem, Constants.Arm.Setpoints.STOWED),
            "zero everything",
            (new SetZeroModeCommand(armSubsystem))
                .alongWith(new ZeroIntakeCommand(intakeSubsystem)),
            "intake",
            new GroundPickupCommand(
                intakeSubsystem,
                outtakeSubsystem,
                armSubsystem,
                () ->
                    intakeLow[0] ? IntakeSubsystem.Modes.INTAKE_LOW : IntakeSubsystem.Modes.INTAKE),
            "squeeze intake",
            new CommandBase() {
              private double lastTime = Timer.getFPGATimestamp();

              @Override
              public void initialize() {
                lastTime = Timer.getFPGATimestamp();
                intakeLow[0] = true;
              }

              @Override
              public boolean isFinished() {
                return Timer.getFPGATimestamp() - lastTime > 0.5;
              }

              @Override
              public void end(boolean interrupted) {
                intakeLow[0] = false;
              }
            },
            "stage outtake",
            new ScoreCommand(outtakeSubsystem, armSubsystem, drivingCubeOuttake.subList(0, 1), 1),
            "stage outtake high",
            new ScoreCommand(
                outtakeSubsystem,
                armSubsystem,
                Constants.SCORE_STEP_MAP.get(NodeType.CUBE.atHeight(Height.HIGH)).subList(0, 1)),
            "stage outtake mid",
            new ScoreCommand(
                outtakeSubsystem,
                armSubsystem,
                Constants.SCORE_STEP_MAP.get(NodeType.CUBE.atHeight(Height.MID)).subList(0, 1)),
            "outtake",
            new ScoreCommand(outtakeSubsystem, armSubsystem, drivingCubeOuttake.subList(1, 2), 1)
                .andThen(
                    new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED)
                        .andThen(
                            new SetOuttakeModeCommand(
                                outtakeSubsystem, OuttakeSubsystem.Modes.OFF))),
            "armbat preload",
            new ArmPositionCommand(armSubsystem, new ArmState(30, 0))
                .andThen(new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED)));

    driverView.add("auto selector", autoSelector).withSize(4, 1).withPosition(7, 0);

    autoDelay =
        driverView
            .add("auto delay", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 15, "block increment", .1))
            .withSize(4, 1)
            .withPosition(7, 1)
            .getEntry();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    double delay = autoDelay.getDouble(0);
    return delay == 0
        ? autoSelector.getSelected()
        : new WaitCommand(delay).andThen(autoSelector.getSelected());
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
