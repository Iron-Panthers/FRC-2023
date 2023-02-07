package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  /** The modes of the intake subsystem */
  public enum Modes {
    MOVE_DOWN(Intake.ARM_HARDSTOP_CURRENT, true),
    BITE(1, false),
    SWALLOW(0.5, false),
    MOVE_UP(2, true),
    EJECT(2, false),
    OFF(2, true);

    // FIXME:   Above values are FAKE!!!

    public final double transitionStatorCurrent;
    private final boolean isArmTransition;

    private Modes(double transitionStatorCurrent, boolean isArmTransition) {
      this.transitionStatorCurrent = transitionStatorCurrent;
      this.isArmTransition = isArmTransition;
    }

    boolean isTransitionReady(double armFilterOutput, double intakeFilterOutput) {
      if (isArmTransition) {
        return armFilterOutput > transitionStatorCurrent;
      } else {
        return intakeFilterOutput > transitionStatorCurrent;
      }
    }
  }

  /** The current mode */
  private Modes mode = Modes.OFF;

  private TalonFX armMotor;

  private TalonFX intakeMotor;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  private LinearFilter armFilter;
  private LinearFilter intakeFilter;

  private double armFilterOutput;
  private double intakeFilterOutput;

  /** Creates a new DrivebaseSubsystem. */
  public IntakeSubsystem() {

    this.armMotor = new TalonFX(Intake.Ports.ARM_MOTOR);
    this.intakeMotor = new TalonFX(Intake.Ports.INTAKE_MOTOR);

    armFilter = LinearFilter.movingAverage(5);
    intakeFilter = LinearFilter.movingAverage(5); // FIXME: tune taps

    armFilterOutput = 0;
    intakeFilterOutput = 0;

    tab.addNumber("armFilter Output", () -> armFilterOutput);
  }

  /**
   * gets the current mode of the intake subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  public void setMode(Modes mode) {
    this.mode = mode;
  }

  public void moveDownPeriodic() {

    armMotor.set(TalonFXControlMode.PercentOutput, 0.3);
  }

  public void bitePeriodic() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0.1);

    if (armFilterOutput > Intake.ARM_HARDSTOP_CURRENT) {
      intakeMotor.set(TalonFXControlMode.PercentOutput, 0.3);
    }
  }

  public void swallowPeriodic() {
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.15);
  }

  public void moveUpPeriodic() {

    armMotor.set(TalonFXControlMode.PercentOutput, -0.3);
  }

  public void ejectPeriodic() {

    intakeMotor.set(TalonFXControlMode.PercentOutput, -0.3);
  }

  public void offPeriodic() {

    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public Modes advanceMode(Modes mode) {

    if (mode.isTransitionReady(armFilterOutput, intakeFilterOutput)) {

      switch (mode) {
        case MOVE_DOWN:
          return Modes.BITE;
        case BITE:
          return Modes.SWALLOW;
        case SWALLOW:
          return Modes.MOVE_UP;
        case MOVE_UP:
        case EJECT:
        case OFF:
          return Modes.OFF;
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
  public void applyMode(Modes mode) {

    switch (mode) {
      case MOVE_DOWN:
        moveDownPeriodic();
        break;
      case BITE:
        bitePeriodic();
        break;
      case SWALLOW:
        swallowPeriodic();
        break;
      case MOVE_UP:
        moveUpPeriodic();
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

    this.armFilterOutput = armFilter.calculate(armMotor.getStatorCurrent());
    this.intakeFilterOutput = intakeFilter.calculate(intakeMotor.getStatorCurrent());

    this.mode = this.advanceMode(mode);

    this.applyMode(mode);
  }
}
