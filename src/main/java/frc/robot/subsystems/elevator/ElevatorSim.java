package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class ElevatorSim implements IElevator {

    public DCMotorSim _gearboxSim;
    public DIOSim _topLimitSwitchSim;
    public DIOSim _bottomLimitSwitchSim;

    public ElevatorSim() {
        _gearboxSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        ElevatorSubsystem.VMap.PositionPID.kV,
                        ElevatorSubsystem.VMap.PositionPID.kA),
                DCMotor.getNeoVortex(1)
                        .withReduction(ElevatorSubsystem.VMap.GearRatio));
        _topLimitSwitchSim = new DIOSim(ElevatorSubsystem.VMap.topLimitSwitchChannel);
        _bottomLimitSwitchSim = new DIOSim(ElevatorSubsystem.VMap.bottomLimitSwitchChannel);
    }

    public void updateInputs(ElevatorInputsAutoLogged inputs) {

        inputs.MotorSpeed = _gearboxSim.getAngularVelocity().in(Units.RotationsPerSecond);
        inputs.TopLimitSwitch = _bottomLimitSwitchSim.getValue();
        inputs.BottomLimitSwitch = _topLimitSwitchSim.getValue();
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
