package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import org.prime.control.ExtendedPIDConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem.EndEffectorMap;

public class EndEffectorReal implements IEndEffector {

    private SparkFlex _intakeMotor;

    private SparkFlex _wristMotor;
    private PIDController _wristPidController;

    private DigitalInput _coralLimitSwitch;

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

    public EndEffectorReal() {
        _coralLimitSwitch = new DigitalInput(EndEffectorMap.LimitSwitchCanID);

        setupIntakeMotor();
        setupWristMotor(EndEffectorMap.WristPID);
    }

    private void setupIntakeMotor() {
        _intakeMotor = new SparkFlex(EndEffectorMap.IntakeMotorCanID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(20);
        config.idleMode(IdleMode.kBrake);
        _intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setupWristMotor(ExtendedPIDConstants pid) {
        _wristMotor = new SparkFlex(EndEffectorMap.WristMotorCanID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(30);
        config.idleMode(IdleMode.kBrake);
        _wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _wristPidController = pid.createPIDController(0.02);

        // TODO: Add the ability to change wrist PID values without having to redeploy
        setWristPID(pid);
    }

    @Override
    public void setWristPID(ExtendedPIDConstants pid) {
        _wristPidController.setP(pid.kP);
        _wristPidController.setI(pid.kI);
        _wristPidController.setD(pid.kD);
    }

    @Override
    public void setWristAngle(Rotation2d angle) {
        // TODO: Ensure this never goes out of bounds
        // TODO: Add a speed limit to the wrist motor for saftey
        var wristOutput = _wristPidController.calculate(getWristAngle().asDegrees(), angle.getDegrees());
        setWristMotorSpeed(wristOutput);
    }

    @Override
    public void setIntakeMotorSpeed(double speed) {
        _intakeMotor.set(speed);
    }

    @Override
    public void stopIntakeMotor() {
        _intakeMotor.stopMotor();
    }

    @Override
    public void stopWristMotor() {
        _wristMotor.stopMotor();
    }

    @Override
    public void stopMotors() {
        _intakeMotor.stopMotor();
        _wristMotor.stopMotor();
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        var intakeMotorSpeed = _intakeMotor.getAbsoluteEncoder().getVelocity();
        var wristMotorSpeed = _wristMotor.getAbsoluteEncoder().getVelocity();
        var limitSwitchState = getLimitSwitchState();

        inputs.IntakeMotorSpeed = intakeMotorSpeed;
        inputs.WristMotorSpeed = wristMotorSpeed;
        inputs.LimitSwitchState = limitSwitchState;
        inputs.EndEffectorAngleDegrees = getWristAngle().asDegrees();
    }

    private void setWristMotorSpeed(double speed) {
        _wristMotor.set(speed);
    }

    private WristAngle getWristAngle() {
        double wristRotations = _wristMotor.getEncoder().getPosition() / EndEffectorMap.GearRatio;

        _wristAngle.update(wristRotations);
        return _wristAngle;
    }

    private boolean getLimitSwitchState() {
        return _coralLimitSwitch.get();
    }
}
