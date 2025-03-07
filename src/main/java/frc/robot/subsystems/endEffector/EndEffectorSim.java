package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem.EndEffectorMap;

public class EndEffectorSim implements IEndEffector {

    private DCMotorSim _endEffectorIntakeMotor;
    private DCMotorSim _endEffectorWristMotor;
    private DIOSim _coralLimitSwitch;
    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    public EndEffectorSim() {
        _endEffectorIntakeMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, EndEffectorMap.GearRatio),
                DCMotor.getNeoVortex(1));
        _endEffectorWristMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, EndEffectorMap.GearRatio),
                DCMotor.getNeoVortex(1));

    }

    @Override
    public void setIntakeMotorSpeed(double speedRadians) {
        _endEffectorIntakeMotor.setAngularVelocity(speedRadians);
    }

    @Override
    public void setWristMotorSpeed(double speedRadians) {
        _endEffectorWristMotor.setAngularVelocity(speedRadians);
    }

    @Override
    public void stopIntakeMotor() {
        _endEffectorIntakeMotor.setAngularVelocity(0);
    }

    @Override
    public void stopWristMotor() {
        _endEffectorWristMotor.setAngularVelocity(0);
    }

    @Override
    public void stopMotors() {
        _endEffectorIntakeMotor.setAngularVelocity(0);
        _endEffectorWristMotor.setAngularVelocity(0);
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        _endEffectorIntakeMotor.update(0.02);
        _endEffectorWristMotor.update(0.02);

        var motorIntakeSpeed = _endEffectorIntakeMotor.getAngularVelocity().magnitude();
        var motorWristSpeed = _endEffectorWristMotor.getAngularVelocity().magnitude();
        var limitSwitchState = getLimitSwitchState();

        inputs.IntakeMotorSpeed = motorIntakeSpeed;
        inputs.WristMotorSpeed = motorWristSpeed;
        inputs.LimitSwitchState = limitSwitchState;
    }

    private boolean getLimitSwitchState() {
        return _coralLimitSwitch.getValue();
    }
}
