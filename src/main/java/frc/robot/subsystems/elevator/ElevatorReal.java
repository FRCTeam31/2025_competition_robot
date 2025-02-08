package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class ElevatorReal implements IElevator {

    public SparkFlex _leftElevatorMotor;
    public SparkFlex _rightElevatorMotor;
    public CANcoder _outputEncoder;
    public DigitalInput _topElevatorLimitSwitch;
    public DigitalInput _bottomElevatorLimitSwitch;

    public ElevatorReal() {
        _leftElevatorMotor = new SparkFlex(ElevatorSubsystem.VMap.leftElevatorMotorCANID, MotorType.kBrushless);
        _rightElevatorMotor = new SparkFlex(ElevatorSubsystem.VMap.rightElevatorMotorCANID, MotorType.kBrushless);

        _outputEncoder = new CANcoder(ElevatorSubsystem.VMap.cancoderCANID);

        _topElevatorLimitSwitch = new DigitalInput(ElevatorSubsystem.VMap.topLimitSwitchChannel);
        _bottomElevatorLimitSwitch = new DigitalInput(ElevatorSubsystem.VMap.bottomLimitSwitchChannel);
    }

    public void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.LeftMotorSpeed = _leftElevatorMotor.get();
        inputs.RightMotorSpeed = _rightElevatorMotor.get();

        inputs.LeftMotorVoltage = _leftElevatorMotor.getBusVoltage() * _leftElevatorMotor.getAppliedOutput();
        inputs.RightMotorVoltage = _rightElevatorMotor.getBusVoltage() * _rightElevatorMotor.getAppliedOutput();
        inputs.ElevatorDistanceMeters = getElevatorDistance();
        inputs.ElevatorSpeedMetersPerSecond = getElevatorSpeedMetersPerSecond();

        inputs.TopLimitSwitch = _topElevatorLimitSwitch.get();
        inputs.BottomLimitSwitch = _bottomElevatorLimitSwitch.get();
    }

    public void setMotorVoltages(double volts) {
        _leftElevatorMotor.setVoltage(volts);
        _rightElevatorMotor.setVoltage(volts);
    }

    public void setMotorSpeeds(double output) {
        var reachedTopLimit = _topElevatorLimitSwitch.get();
        var reachedBottomLimit = _bottomElevatorLimitSwitch.get();
        var speedIsUp = output > 0;
        var speedIsDown = output < 0;

        if ((speedIsUp && reachedTopLimit) || (speedIsDown && reachedBottomLimit)) {
            DriverStation.reportWarning("[ELEVATOR] Reached Elevator Limit!", false);
            return;
        }

        _leftElevatorMotor.set(output);
        _rightElevatorMotor.set(output);
    }

    public void stopMotors() {
        _leftElevatorMotor.stopMotor();
        _rightElevatorMotor.stopMotor();
    }

    public double getElevatorDistance() {
        var rotations = _outputEncoder.getPosition().getValueAsDouble();
        return rotations * Math.PI * ElevatorSubsystem.VMap.OutputSprocketDiameterMeters;
    }

    public double getElevatorSpeedMetersPerSecond() {
        var speedRPS = _outputEncoder.getVelocity().getValueAsDouble();
        return speedRPS * Math.PI * ElevatorSubsystem.VMap.OutputSprocketDiameterMeters;
    }
}
