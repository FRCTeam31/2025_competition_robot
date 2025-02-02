package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.maps.AlgaeIntakeMap;

public class AlgaeIntakeIOSim implements IAlgaeIntakeIO {

    public AlgaeIntakeInputs m_inputs = new AlgaeIntakeInputs();

    public DoubleSolenoidSim leftAlgaeSolenoidSim;
    public DoubleSolenoidSim rightAlgaeSolenoidSim;
    public DCMotorSim algaeMotorSim;

    public AlgaeIntakeIOSim() {
        leftAlgaeSolenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.CTREPCM,
                AlgaeIntakeMap.leftAlgaeSolenoidForwardChannel, AlgaeIntakeMap.leftAlgaeSolenoidReverseChannel);
        rightAlgaeSolenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.CTREPCM,
                AlgaeIntakeMap.rightAlgaeSolenoidForwardChannel, AlgaeIntakeMap.rightAlgaeSolenoidReverseChannel);
        algaeMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, AlgaeIntakeMap.algaeIntakeGearRatio),
                DCMotor.getNeoVortex(1));
    }

    public AlgaeIntakeInputs getInputs() {
        Value leftSolenoidPosition = leftAlgaeSolenoidSim.get();
        Value rightSolenoidPosition = rightAlgaeSolenoidSim.get();
        double motorSpeed = algaeMotorSim.getAngularVelocityRadPerSec();

        m_inputs.AlgaeMotorSpeed = motorSpeed;
        m_inputs.AlgaeLeftSolenoidPosition = leftSolenoidPosition;
        m_inputs.AlgaeRightSolenoidPosition = rightSolenoidPosition;

        return m_inputs;
    }

    public void setAlgaeMotorSpeed(double speed) {
        algaeMotorSim.setAngularVelocity(speed);
    }

    public void stopMotors() {
        algaeMotorSim.setAngularVelocity(0);
    }

    public void setAlgaeIntakePosition(Value algaeSolenoidPosition) {
        leftAlgaeSolenoidSim.set(algaeSolenoidPosition);
        rightAlgaeSolenoidSim.set(algaeSolenoidPosition);
    }
}
