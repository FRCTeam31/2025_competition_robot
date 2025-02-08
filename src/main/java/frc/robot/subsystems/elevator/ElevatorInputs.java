package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorInputs {

    public double LeftMotorSpeed = 0;
    public double RightMotorSpeed = 0;
    public DigitalInput TopLimitSwitch;
    public DigitalInput BottomLimitSwitch;

    public ElevatorInputs() {

    }

    public ElevatorInputs(double leftMotorSpeed, double rightMotorSpeed, DigitalInput topLimitSwitch,
            DigitalInput bottomLimitSwitch) {

        LeftMotorSpeed = leftMotorSpeed;
        RightMotorSpeed = rightMotorSpeed;
        TopLimitSwitch = topLimitSwitch;
        BottomLimitSwitch = bottomLimitSwitch;

    }
}
