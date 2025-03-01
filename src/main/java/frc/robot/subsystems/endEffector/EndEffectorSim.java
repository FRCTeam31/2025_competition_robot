package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.subsystems.drivetrain.SwerveMap;

public class EndEffectorSim implements IEndEffector {

    private DCMotorSim _endEffectorMotor;
    private DIOSim _coralLimitSwitch;
    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    public EndEffectorSim() {
        _endEffectorMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, EndEffectorMap.GearRatio),
                DCMotor.getNeoVortex(1));

    }

    public void setMotorSpeed(double speedRadians) {
        _endEffectorMotor.setAngularVelocity(speedRadians);
    }

    public void stopMotors() {
        _endEffectorMotor.setAngularVelocity(0);
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        _endEffectorMotor.update(0.02);

        var motorSpeed = _endEffectorMotor.getAngularVelocity();
        var limitSwitchState = getLimitSwitchState();

        inputs.MotorSpeed = motorSpeed;
        inputs.LimitSwitchState = limitSwitchState;
    }

    private boolean getLimitSwitchState() {
        return _coralLimitSwitch.getValue();
    }
}
