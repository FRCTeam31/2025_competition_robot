package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class ElevatorReal implements IElevator {

    public SparkFlex _elevatorMotor;
    public CANcoder _outputEncoder;
    public DigitalInput _topElevatorLimitSwitch;
    public DigitalInput _bottomElevatorLimitSwitch;

    public ElevatorReal() {
        _elevatorMotor = new SparkFlex(ElevatorSubsystem.VMap.leftElevatorMotorCANID, MotorType.kBrushless);

        _outputEncoder = new CANcoder(ElevatorSubsystem.VMap.cancoderCANID);

        _topElevatorLimitSwitch = new DigitalInput(ElevatorSubsystem.VMap.topLimitSwitchChannel);
        _bottomElevatorLimitSwitch = new DigitalInput(ElevatorSubsystem.VMap.bottomLimitSwitchChannel);
    }

    public void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.MotorSpeed = _elevatorMotor.get();

        inputs.MotorVoltage = _elevatorMotor.getBusVoltage() * _elevatorMotor.getAppliedOutput();
        inputs.ElevatorDistanceMeters = getElevatorDistance();
        inputs.ElevatorSpeedMetersPerSecond = getElevatorSpeedMetersPerSecond();

        inputs.TopLimitSwitch = _topElevatorLimitSwitch.get();
        inputs.BottomLimitSwitch = _bottomElevatorLimitSwitch.get();
    }

    public void setMotorVoltages(double volts) {
        _elevatorMotor.setVoltage(volts);
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

        _elevatorMotor.set(output);
    }

    public void stopMotors() {
        _elevatorMotor.stopMotor();
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
