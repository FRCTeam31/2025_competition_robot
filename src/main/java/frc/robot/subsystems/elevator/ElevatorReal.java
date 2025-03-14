package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMap;

public class ElevatorReal implements IElevator {

    private SparkFlex _leftElevatorMotor;
    private SparkFlex _rightElevatorMotor;
    private DigitalInput _topElevatorLimitSwitch;
    private DigitalInput _bottomElevatorLimitSwitch;
    private CANcoder _elevatorEncoder;

    public ElevatorReal() {
        setupElevatorMotors();
        _topElevatorLimitSwitch = new DigitalInput(ElevatorMap.TopLimitSwitchChannel);
        _bottomElevatorLimitSwitch = new DigitalInput(ElevatorMap.BottomLimitSwitchChannel);
        _elevatorEncoder = new CANcoder(ElevatorMap.ElevatorEncoderCANID);
    }

    public void setupElevatorMotors() {
        _leftElevatorMotor = new SparkFlex(ElevatorMap.LeftElevatorMotorCANID, MotorType.kBrushless);
        _rightElevatorMotor = new SparkFlex(ElevatorMap.RightElevatorMotorCANID, MotorType.kBrushless);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        leftMotorConfig.smartCurrentLimit(40);
        rightMotorConfig.smartCurrentLimit(40);
        leftMotorConfig.openLoopRampRate(2);
        rightMotorConfig.openLoopRampRate(2);

        leftMotorConfig.idleMode(IdleMode.kBrake);
        rightMotorConfig.idleMode(IdleMode.kBrake);

        rightMotorConfig.follow(ElevatorMap.LeftElevatorMotorCANID, true);
        _leftElevatorMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _rightElevatorMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.MotorSpeed = _leftElevatorMotor.get();
        inputs.MotorVoltage = _leftElevatorMotor.getBusVoltage() * _leftElevatorMotor.getAppliedOutput();
        inputs.ElevatorDistanceMeters = getElevatorDistance();
        inputs.ElevatorSpeedMetersPerSecond = getElevatorSpeedMetersPerSecond();
        inputs.TopLimitSwitch = _topElevatorLimitSwitch.get();
        inputs.BottomLimitSwitch = _bottomElevatorLimitSwitch.get();
    }

    public void setMotorVoltages(double volts) {
        var deadbandedVolts = MathUtil.applyDeadband(volts, 0.05);
        _leftElevatorMotor.setVoltage(deadbandedVolts);
    }

    @Override
    public void setMotorSpeeds(double output) {
        // right elevator motor follows the left motor
        var deadbandedSpeed = MathUtil.applyDeadband(output, 0.06);
        _leftElevatorMotor.set(deadbandedSpeed);
    }

    @Override
    public void stopMotors() {
        _leftElevatorMotor.stopMotor();
        _rightElevatorMotor.stopMotor();
    }

    public double getElevatorDistance() {
        var rotations = _elevatorEncoder.getPosition().getValueAsDouble();
        return rotations * Math.PI * ElevatorMap.OutputSprocketDiameterMeters;
    }

    public double getElevatorSpeedMetersPerSecond() {
        var speedRPS = _elevatorEncoder.getVelocity().getValueAsDouble() / ElevatorMap.GearRatio;
        return speedRPS * Math.PI * ElevatorMap.OutputSprocketDiameterMeters;
    }
}