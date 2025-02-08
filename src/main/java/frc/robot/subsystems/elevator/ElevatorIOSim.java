package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Enums.ElevatorLocation;
import frc.robot.maps.ElevatorMap;

public class ElevatorIOSim implements IElevatorIO {

    private ElevatorInputs _inputs = new ElevatorInputs();
    public DCMotorSim leftElevatorMotor;
    public DCMotorSim rightElevatorMotor;

    public ElevatorIOSim() {
        leftElevatorMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, ElevatorMap.elevatorGearRatio),
                DCMotor.getNeoVortex(1));
        rightElevatorMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, ElevatorMap.elevatorGearRatio),
                DCMotor.getNeoVortex(1));
    }

    public ElevatorInputs getInputs() {
        double leftMotorSpeed = leftElevatorMotor.getAngularVelocity().in(Units.RotationsPerSecond);
        double rightMotorSpeed = leftElevatorMotor.getAngularVelocity().in(Units.RotationsPerSecond);

        _inputs.LeftMotorSpeed = leftMotorSpeed;
        _inputs.RightMotorSpeed = rightMotorSpeed;

        return _inputs;
    }

    public void SetElevatorPosition(ElevatorLocation location) {

    }

    public void StopMotors() {
        leftElevatorMotor.setAngularVelocity(0);
        rightElevatorMotor.setAngularVelocity(0);
    }
}
