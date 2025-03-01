package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem.AlgaeIntakeMap;

public class AlgaeIntakeSim implements IAlgaeIntake {
    public DoubleSolenoidSim leftAlgaeSolenoidSim;
    public DoubleSolenoidSim rightAlgaeSolenoidSim;
    public DCMotorSim algaeMotorSim;

    public AlgaeIntakeSim() {
        leftAlgaeSolenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.CTREPCM,
                AlgaeIntakeMap.leftAlgaeSolenoidForwardChannel, AlgaeIntakeMap.leftAlgaeSolenoidReverseChannel);
        rightAlgaeSolenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.CTREPCM,
                AlgaeIntakeMap.rightAlgaeSolenoidForwardChannel, AlgaeIntakeMap.rightAlgaeSolenoidReverseChannel);
        algaeMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, AlgaeIntakeMap.algaeIntakeGearRatio),
                DCMotor.getNeoVortex(1));
    }

    @Override
    public void updateInputs(AlgaeIntakeInputsAutoLogged inputs) {
        inputs.AlgaeLeftSolenoidPosition = leftAlgaeSolenoidSim.get();
        inputs.AlgaeRightSolenoidPosition = rightAlgaeSolenoidSim.get();
        inputs.AlgaeMotorSpeed = algaeMotorSim.getAngularVelocity().in(Units.RotationsPerSecond);
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
