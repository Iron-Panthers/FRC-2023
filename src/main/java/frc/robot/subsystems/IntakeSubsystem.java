package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  /** The IntakeModes of the intake subsystem */
  public enum IntakeModes {
    DEPLOY(Intake.ARM_HARDSTOP_CURRENT, Intake.TransitionTimes.DEPLOY_TIMING),
    INTAKE(1, Intake.TransitionTimes.INTAKE_TIMING),
    RETRACT(2, Intake.TransitionTimes.RETRACT_TIMING),
    EJECT(2, Intake.TransitionTimes.EJECT_TIMING),
    OFF(2, 0);

    // FIXME:   Above values are FAKE!!!

    public final double transitionStatorCurrent;
    public final double transitionTime;

    private IntakeModes(double transitionStatorCurrent, double transitionTime) {
      this.transitionStatorCurrent = transitionStatorCurrent;
      this.transitionTime = transitionTime;
    }

  }

  /** The current mode */
  private IntakeModes mode = IntakeModes.OFF;

  private TalonFX armMotor;

  private TalonFX intakeMotor;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  private LinearFilter armFilter;
  private LinearFilter intakeFilter;

  // private double armFilterOutput;
  // private double intakeFilterOutput;

  private double previousTransitionTime;

  /** Creates a new DrivebaseSubsystem. */
  public IntakeSubsystem() {

    this.armMotor = new TalonFX(Intake.Ports.ARM_MOTOR);
    this.intakeMotor = new TalonFX(Intake.Ports.INTAKE_MOTOR);

    armFilter = LinearFilter.movingAverage(5);
    intakeFilter = LinearFilter.movingAverage(5); // FIXME: tune taps

    // armFilterOutput = 0;
    // intakeFilterOutput = 0;

    previousTransitionTime = 0;

    // tab.addNumber("armFilter Output", () -> armFilterOutput);
  }

  /**
   * gets the current mode of the intake subsystem state machine
   *
   * @return the current mode
   */
  public IntakeModes getMode() {
    return mode;
  }

  public void setMode(IntakeModes mode) {

    if(mode != this.mode) {
      previousTransitionTime = Timer.getFPGATimestamp();
    }

    this.mode = mode;
  }

  public void deployPeriodic() {

    armMotor.set(TalonFXControlMode.PercentOutput, 0.09);
  }

  public void intakePeriodic() {
    // armMotor.set(TalonFXControlMode.PercentOutput, 0.1);

    // if (armFilterOutput > Intake.ARM_HARDSTOP_CURRENT) {
    //   intakeMotor.set(TalonFXControlMode.PercentOutput, 0.3);
    // }
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.09);
  }

  public void retractPeriodic() {

    armMotor.set(TalonFXControlMode.PercentOutput, -0.09);
  }

  public void ejectPeriodic() {

    intakeMotor.set(TalonFXControlMode.PercentOutput, -0.09);
  }

  public void offPeriodic() {

    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.09);
  }

  public IntakeModes advanceMode(IntakeModes mode) {

    if (Timer.getFPGATimestamp() - previousTransitionTime >= mode.transitionTime) {

      switch (mode) {
        case DEPLOY:
          return IntakeModes.INTAKE;
        case INTAKE:
          return IntakeModes.RETRACT;
        case RETRACT:
          return IntakeModes.OFF;
        case EJECT:
          return IntakeModes.OFF;
        case OFF:
          return IntakeModes.OFF;
      }
    }

    return mode;
  }

  /**
   * Based on the current Mode of the intake subsystem, perform the mode-specific logic such as
   * writing outputs (may vary per mode).
   *
   * @param mode The mode to use (should use the current mode value)
   */
  public void applyMode(IntakeModes mode) {

    switch (mode) {
      case DEPLOY:
        deployPeriodic();
        break;
      case INTAKE:
        intakePeriodic();
        break;
      case RETRACT:
        retractPeriodic();
        break;
      case EJECT:
        ejectPeriodic();
        break;
      case OFF:
        offPeriodic();
        break;
    }
  }

  @Override
  public void periodic() {

    // armFilterOutput = armFilter.calculate(armMotor.getStatorCurrent());
    // intakeFilterOutput = intakeFilter.calculate(intakeMotor.getStatorCurrent());

    // mode = advanceMode(mode);

    applyMode(mode);
  }
}
