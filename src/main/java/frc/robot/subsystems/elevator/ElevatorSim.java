package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorSim implements IElevator {

    public DCMotorSim _gearboxSim;

    public ElevatorSim() {
        _gearboxSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        ElevatorSubsystem.VMap.PositionPID.kV,
                        ElevatorSubsystem.VMap.PositionPID.kA),
                DCMotor.getNeoVortex(1)
                        .withReduction(ElevatorSubsystem.VMap.GearRatio));
    }

    public void updateInputs(ElevatorInputsAutoLogged inputs) {

        inputs.MotorSpeed = _gearboxSim.getAngularVelocity().in(Units.RotationsPerSecond);
        inputs.TopLimitSwitch = false;
        inputs.BottomLimitSwitch = false;
    }

    public void setMotorVoltages(double volts) {
        _gearboxSim.setInputVoltage(volts);
    }

    public void setMotorSpeeds(double output) {
        var voltage = output * 12.0;
        _gearboxSim.setInputVoltage(voltage);
    }

    public void stopMotors() {
        _gearboxSim.setAngularVelocity(0);
    }
}
