package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorInputs {

    public double LeftMotorSpeed = 0;
    public double RightMotorSpeed = 0;
    public double LeftMotorVoltage = 0;
    public double RightMotorVoltage = 0;
    public double ElevatorDistanceMeters = 0;
    public double ElevatorSpeedMetersPerSecond = 0;
    public boolean TopLimitSwitch;
    public boolean BottomLimitSwitch;

}
