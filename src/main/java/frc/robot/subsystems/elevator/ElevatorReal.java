package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMap;

public class ElevatorReal implements IElevator {

    private SparkFlex _elevatorMotor;
    private DigitalInput _topElevatorLimitSwitch;
    private DigitalInput _bottomElevatorLimitSwitch;
    private RelativeEncoder _outputEncoder;

    public ElevatorReal() {
        _elevatorMotor = new SparkFlex(ElevatorMap.leftElevatorMotorCANID, MotorType.kBrushless);
        _topElevatorLimitSwitch = new DigitalInput(ElevatorMap.topLimitSwitchChannel);
        _bottomElevatorLimitSwitch = new DigitalInput(ElevatorMap.bottomLimitSwitchChannel);
        _outputEncoder = _elevatorMotor.getEncoder();
    }

    @Override
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

    @Override
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

    @Override
    public void stopMotors() {
        _elevatorMotor.stopMotor();
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
