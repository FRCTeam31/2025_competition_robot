package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMap;

public class ElevatorSim implements IElevator {

    public DCMotorSim _gearboxSim;
    public DIOSim _topLimitSwitchSim;
    public DIOSim _bottomLimitSwitchSim;

    public ElevatorSim() {
        _gearboxSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        ElevatorMap.PositionPID.kV,
                        ElevatorMap.PositionPID.kA),
                DCMotor.getNeoVortex(1)
                        .withReduction(ElevatorMap.GearRatio));
        _topLimitSwitchSim = new DIOSim(ElevatorMap.TopLimitSwitchChannel);
        _bottomLimitSwitchSim = new DIOSim(ElevatorMap.BottomLimitSwitchChannel);
    }

    @Override
    public void updateInputs(ElevatorInputsAutoLogged inputs) {

        inputs.MotorSpeed = _gearboxSim.getAngularVelocity().in(Units.RotationsPerSecond);
        inputs.TopLimitSwitch = _bottomLimitSwitchSim.getValue();
        inputs.BottomLimitSwitch = _topLimitSwitchSim.getValue();
    }

    @Override
    public void setMotorVoltages(double volts) {
        _gearboxSim.setInputVoltage(volts);
    }

    @Override
    public void setMotorSpeeds(double output) {
        var voltage = output * 12.0;
        _gearboxSim.setInputVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        _gearboxSim.setAngularVelocity(0);
    }

    @Override
    public void resetElevatorPosition() {

    }
}
