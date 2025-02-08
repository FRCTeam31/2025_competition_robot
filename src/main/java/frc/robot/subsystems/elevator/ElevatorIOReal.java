package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Enums.ElevatorLocation;
import frc.robot.maps.ElevatorMap;

public class ElevatorIOReal implements IElevatorIO {

    private ElevatorInputs _inputs = new ElevatorInputs();

    public SparkFlex leftElevatorMotor;
    public SparkFlex rightElevatorMotor;
    // public DigitalInput topElevatorLimitSwitch;
    // public DigitalInput bottomElevatorLimitSwitch;

    public ElevatorIOReal() {
        leftElevatorMotor = new SparkFlex(ElevatorMap.leftElevatorMotorCANID, MotorType.kBrushless);
        rightElevatorMotor = new SparkFlex(ElevatorMap.rightElevatorMotorCANID, MotorType.kBrushless);
        // topElevatorLimitSwitch = new DigitalInput(ElevatorMap.topLimitSwitchChannel);
        // bottomElevatorLimitSwitch = new DigitalInput(ElevatorMap.bottomLimitSwitchChannel);

    }

    public ElevatorInputs getInputs() {
        double leftMotorSpeed = leftElevatorMotor.get();
        double rightMotorSpeed = rightElevatorMotor.get();

        _inputs.LeftMotorSpeed = leftMotorSpeed;
        _inputs.RightMotorSpeed = rightMotorSpeed;

        return _inputs;
    }

    public void SetElevatorPosition(ElevatorLocation location) {

    }

    public void StopMotors() {
        leftElevatorMotor.stopMotor();
        rightElevatorMotor.stopMotor();
    }

}
