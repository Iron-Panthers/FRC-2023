package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorTestSubsystem extends SubsystemBase {

private TalonFX testMotor, testMotor2;

    private double motorPower = 0;

    public ElevatorTestSubsystem() {
        testMotor = new TalonFX(10);
        testMotor2 = new TalonFX(11); //FIXME Placeholder

        testMotor.configFactoryDefault();
    }


    public void setMotorPower(double power) {
        motorPower = power;
    }

    @Override
    public void periodic() {
        testMotor.set(ControlMode.PercentOutput, motorPower);
        testMotor2.set(ControlMode.PercentOutput, motorPower);
    }
}