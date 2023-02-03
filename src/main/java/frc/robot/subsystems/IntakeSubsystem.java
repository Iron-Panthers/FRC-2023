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
    MOVE_DOWN(Intake.ARM_HARDSTOP_CURRENT),
    INTAKE(1),
    MOVE_UP(2),
    DROP(1),
    EJECT(2),
    OFF(2);

    // FIXME:   Above values are FAKE!!!

    public final double transitionStatorCurrent;

    private Modes(double transitionStatorCurrent) {
      this.transitionStatorCurrent = transitionStatorCurrent;
    }
  }

  /** The current mode */
  private Modes mode = Modes.OFF;

  private TalonFX armMotor;

  private TalonFX intakeMotor;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  private LinearFilter filter;

  private double filterOutput;

  /** Creates a new DrivebaseSubsystem. */
  public IntakeSubsystem() {

    this.armMotor = new TalonFX(Intake.Ports.ARM_MOTOR);
    this.intakeMotor = new TalonFX(Intake.Ports.INTAKE_MOTOR);

    filter = LinearFilter.movingAverage(5);

    filterOutput = 0;

    tab.addNumber("Filter Output", () -> filterOutput);
  }

  /**
   * gets the current mode of the intake subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  public void moveDownPeriodic() {

    armMotor.set(TalonFXControlMode.PercentOutput, 0.3);
  }

  public void intakePeriodic() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0.1);

    if (filterOutput > Intake.ARM_HARDSTOP_CURRENT) {
      intakeMotor.set(TalonFXControlMode.PercentOutput, 0.3);
    }
  }

  public void moveUpPeriodic() {

    armMotor.set(TalonFXControlMode.PercentOutput, -0.3);
  }

  public void dropPeriodic() {

    intakeMotor.set(TalonFXControlMode.PercentOutput, -0.3);
  }

  public void ejectPeriodic() {

    intakeMotor.set(TalonFXControlMode.PercentOutput, -0.5);
  }

  public void offPeriodic() {

    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public Modes advanceMode(Modes mode) {

    if (filterOutput > mode.transitionStatorCurrent) {

      switch (mode) {
        case MOVE_DOWN:
        case EJECT:
          return Modes.INTAKE;
        case MOVE_UP:
          return Modes.DROP;
        case DROP:
        case OFF:
          return Modes.OFF;
        case INTAKE:
          return mode;
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
      case INTAKE:
        intakePeriodic();
        break;
      case MOVE_UP:
        moveUpPeriodic();
        break;
      case DROP:
        dropPeriodic();
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

    this.filterOutput = filter.calculate(armMotor.getStatorCurrent());

    this.mode = this.advanceMode(mode);

    this.applyMode(mode);
  }
}
