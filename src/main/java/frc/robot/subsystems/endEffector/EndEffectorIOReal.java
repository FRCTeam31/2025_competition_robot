package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorIOReal implements IEndEffectorIO {

    private SparkFlex endEffectorMotor;
    private EndEffectorIOInputs m_inputs = new EndEffectorIOInputs();

    public EndEffectorIOReal() {
        endEffectorMotor = new SparkFlex(EndEffectorMap.endEffectorCanID, MotorType.kBrushless);

    }

    public EndEffectorIOInputs getInputs() {
        double motorSpeed = endEffectorMotor.getEncoder().getVelocity();
        m_inputs.VelocityRadPerSec = motorSpeed;

        return m_inputs;
    }

    public void setMotorSpeed(double speed) {
        endEffectorMotor.set(speed);
    }

    public void stopMotors() {
        endEffectorMotor.stopMotor();
    }
}
