package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import org.prime.control.ExtendedPIDConstants;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffectorReal implements IEndEffector {
    private CANrange _distanceSensor;

    private SparkFlex _intakeMotor;

    private SparkFlex _wristMotor;

    private DigitalInput _coralLimitSwitch;

    public EndEffectorReal() {
        _distanceSensor = new CANrange(EndEffectorMap.DistanceSensorCanID);
        _coralLimitSwitch = new DigitalInput(EndEffectorMap.LimitSwitchDIOChannel);

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
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        var intakeMotorSpeed = _intakeMotor.getAbsoluteEncoder().getVelocity();
        var wristMotorSpeed = _wristMotor.getAbsoluteEncoder().getVelocity();
        var limitSwitchState = getLimitSwitchState();

        inputs.IntakeMotorSpeed = intakeMotorSpeed;
        inputs.WristMotorSpeed = wristMotorSpeed;
        inputs.CoralLimitSwitchState = limitSwitchState;
        inputs.EndEffectorAngleDegrees = getWristAngle();
        inputs.RangeSensorDistance = _distanceSensor.getDistance().getValueAsDouble();
    }

    @Override
    public void setIntakeSpeed(double speed) {
        // _intakeMotor.set(speed);
    }

    @Override
    public void setWristSpeed(double speed) {
        _wristMotor.set(speed);
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

    private double getWristAngle() {
        double wristRotations = _wristMotor.getEncoder().getPosition() / EndEffectorMap.WristGearRatio;

        return Rotation2d.fromRotations(wristRotations).getDegrees();
    }

    private boolean getLimitSwitchState() {
        return _coralLimitSwitch.get();
    }
}
