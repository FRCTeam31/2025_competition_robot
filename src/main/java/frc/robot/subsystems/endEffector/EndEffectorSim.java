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
    private PIDController _wristPIDController;

    private DIOSim _coralLimitSwitch;

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    private WristAngle _wristAngle = new WristAngle(0);

    private class WristAngle {
        private double _rotations;

        public WristAngle(double rotations) {
            _rotations = rotations;
        }

        public double asDegrees() {
            return Rotation2d.fromRotations(_rotations).getDegrees();
        }

        public Rotation2d asRotation2d() {
            return Rotation2d.fromRotations(_rotations);
        }

        public void update(double rotations) {
            _rotations = rotations;
        }
    }

    public EndEffectorSim() {
        _coralLimitSwitch = new DIOSim(EndEffectorMap.LimitSwitchCanID);

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

        _wristPIDController = pid.createPIDController(0.02);

        // TODO: Add the ability to change wrist PID values without having to restart sim
        setWristPID(pid);
    }

    @Override
    public void setWristPID(ExtendedPIDConstants pid) {
        _wristPIDController.setP(pid.kP);
        _wristPIDController.setI(pid.kI);
        _wristPIDController.setD(pid.kD);
    }

    @Override
    public void setWristAngle(Rotation2d angle) {
        var wristOutput = _wristPIDController.calculate(getWristAngle().asDegrees(), angle.getDegrees());
        setWristMotorSpeed(wristOutput);
    }

    @Override
    public void setIntakeMotorSpeed(double speedRadians) {
        _intakeMotor.setAngularVelocity(speedRadians);
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

    private void setWristMotorSpeed(double speedRadians) {
        _wristMotor.setAngularVelocity(speedRadians);
    }

    private WristAngle getWristAngle() {
        double wristRotations = _wristMotor.getAngularPositionRotations() / EndEffectorMap.GearRatio;

        _wristAngle.update(wristRotations);
        return _wristAngle;
    }

    private boolean getLimitSwitchState() {
        return _coralLimitSwitch.getValue();
    }
}
