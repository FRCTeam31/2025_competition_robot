package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.maps.DriveMap;
import frc.robot.maps.EndEffectorMap;

public class EndEffectorIOSim implements IEndEffectorIO {

    public DCMotorSim endEffectorMotor;

    public EndEffectorIOSim() {
        endEffectorMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, DriveMap.DriveGearRatio),
                DCMotor.getNeoVortex(1));

    }

    public void getInputs() {
        double motorSpeed = endEffectorMotor.g

    }

    public void setMotorSpeed(double speed) {
        endEffectorMotor.set(speed);
    // }

    public void stopMotors() {

    }
}
