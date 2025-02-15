package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorInputs {

    public double MotorSpeed = 0;
    public double MotorVoltage = 0;
    public double ElevatorDistanceMeters = 0;
    public double ElevatorSpeedMetersPerSecond = 0;
    public boolean TopLimitSwitch;
    public boolean BottomLimitSwitch;

}
