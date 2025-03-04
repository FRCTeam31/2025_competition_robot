package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMap;

public class ElevatorReal implements IElevator {

    private SparkFlex _leftElevatorMotor;
    private SparkFlex _rightElevatorMotor;

    private DigitalInput _topElevatorLimitSwitch;
    private DigitalInput _bottomElevatorLimitSwitch;
    private RelativeEncoder _outputEncoder;

    public ElevatorReal() {
        _leftElevatorMotor = new SparkFlex(ElevatorMap.leftElevatorMotorCANID, MotorType.kBrushless);
        _topElevatorLimitSwitch = new DigitalInput(ElevatorMap.topLimitSwitchChannel);
        _bottomElevatorLimitSwitch = new DigitalInput(ElevatorMap.bottomLimitSwitchChannel);
        _outputEncoder = _leftElevatorMotor.getEncoder();
    }

    public void motorConfig() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        leftMotorConfig.follow(ElevatorMap.rightElevatorMotorCANID, true);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        rightMotorConfig.idleMode(IdleMode.kBrake);

        _leftElevatorMotor = new SparkFlex(ElevatorMap.leftElevatorMotorCANID, MotorType.kBrushless);
        _rightElevatorMotor = new SparkFlex(ElevatorMap.rightElevatorMotorCANID, MotorType.kBrushless);
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
        _leftElevatorMotor.setVoltage(volts);
    }

    @Override
    public void setMotorSpeeds(double output) {

        _leftElevatorMotor.set(output);
        _rightElevatorMotor.set(output);
    }

    @Override
    public void stopMotors() {
        _leftElevatorMotor.stopMotor();
    }

    public double getElevatorDistance() {
        var rotations = _outputEncoder.getPosition();
        return rotations * Math.PI * ElevatorMap.OutputSprocketDiameterMeters;
    }

    public double getElevatorSpeedMetersPerSecond() {
        var speedRPS = _outputEncoder.getVelocity();
        return speedRPS * Math.PI * ElevatorMap.OutputSprocketDiameterMeters;
    }
}
