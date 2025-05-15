package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Kilograms;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorSim implements IElevator {

    private edu.wpi.first.wpilibj.simulation.ElevatorSim _elevatorSim;

    public ElevatorSim() {
        _elevatorSim = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
                DCMotor.getNeoVortex(2),
                ElevatorMap.GearRatio,
                Units.Pounds.of(3).in(Kilograms),
                ElevatorMap.OutputSprocketDiameterMeters,
                0,
                ElevatorMap.MaxHeight,
                true,
                0,
                0, 0);
    }

    @Override
    public void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.DistanceMeters = _elevatorSim.getPositionMeters();
        inputs.SpeedMPS = _elevatorSim.getVelocityMetersPerSecond();
        inputs.TopLimitSwitch = _elevatorSim.hasHitUpperLimit();
        inputs.BottomLimitSwitch = _elevatorSim.hasHitUpperLimit();
    }

    @Override
    public void setMotorVoltages(double volts) {
        Logger.recordOutput("Elevator/MotorOutputVolts", volts);
        _elevatorSim.setInputVoltage(volts);
    }

    @Override
    public void setMotorSpeeds(double output) {
        var volts = output * RobotController.getBatteryVoltage();
        setMotorVoltages(volts);
    }

    @Override
    public void stopMotors() {
        setMotorVoltages(0);
    }

    @Override
    public void resetEncoderPos() {
        _elevatorSim.setState(0, _elevatorSim.getVelocityMetersPerSecond());
    }
}
