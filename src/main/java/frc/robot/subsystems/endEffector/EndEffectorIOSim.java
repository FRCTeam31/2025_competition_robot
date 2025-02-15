package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drivetrain.SwerveMap;

public class EndEffectorIOSim implements IEndEffectorIO {

    public DCMotorSim endEffectorMotor;
    public EndEffectorIOInputs m_inputs = new EndEffectorIOInputs();

    public EndEffectorIOSim() {
        endEffectorMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, SwerveMap.DriveGearRatio),
                DCMotor.getNeoVortex(1));

    }

    public EndEffectorIOInputs getInputs() {
        double motorSpeed = endEffectorMotor.getAngularVelocityRadPerSec();
        m_inputs.VelocityRadPerSec = motorSpeed;
        return m_inputs;
    }

    public void setMotorSpeed(double speedRadians) {
        endEffectorMotor.setAngularVelocity(speedRadians);
    }

    public void stopMotors() {
        endEffectorMotor.setAngularVelocity(0);
    }
}
