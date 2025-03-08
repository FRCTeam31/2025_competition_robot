package frc.robot.subsystems.endEffector;

import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem.EndEffectorMap;

public class EndEffectorSim implements IEndEffector {

    private DCMotorSim _intakeMotor;

    private DCMotorSim _wristMotor;

    private DIOSim _coralLimitSwitch;

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    public EndEffectorSim() {
        _coralLimitSwitch = new DIOSim(EndEffectorMap.LimitSwitchDIOChannel);

        setupIntakeMotor();
        setupWristMotor(EndEffectorMap.WristPID);
    }

    // TODO: Update the gear ratios in these two methods once we have verified which motor the gear ratio is for

    private void setupIntakeMotor() {
        _intakeMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, EndEffectorMap.GearRatio),
                DCMotor.getNeoVortex(1));
    }

    private void setupWristMotor(ExtendedPIDConstants pid) {
        _wristMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, EndEffectorMap.GearRatio),
                DCMotor.getNeoVortex(1));

    }

    @Override
    public void setIntakeSpeed(double speedRadians) {
        _intakeMotor.setAngularVelocity(speedRadians);
    }

    @Override
    public void setWristSpeed(double speedRadians) {
        _wristMotor.setAngularVelocity(speedRadians);
    }

    @Override
    public void stopIntakeMotor() {
        _intakeMotor.setAngularVelocity(0);
    }

    @Override
    public void stopWristMotor() {
        _wristMotor.setAngularVelocity(0);
    }

    @Override
    public void stopMotors() {
        _intakeMotor.setAngularVelocity(0);
        _wristMotor.setAngularVelocity(0);
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        _intakeMotor.update(0.02);
        _wristMotor.update(0.02);

        var motorIntakeSpeed = _intakeMotor.getAngularVelocity().magnitude();
        var motorWristSpeed = _wristMotor.getAngularVelocity().magnitude();
        var limitSwitchState = getLimitSwitchState();

        inputs.IntakeMotorSpeed = motorIntakeSpeed;
        inputs.WristMotorSpeed = motorWristSpeed;
        inputs.LimitSwitchState = limitSwitchState;
    }

    private double getWristAngle() {
        double wristRotations = _wristMotor.getAngularPositionRotations() / EndEffectorMap.GearRatio;

        return Rotation2d.fromRotations(wristRotations).getDegrees();
    }

    private boolean getLimitSwitchState() {
        return _coralLimitSwitch.getValue();
    }
}
