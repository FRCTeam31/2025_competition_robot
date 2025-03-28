package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class EndEffectorInputs {
    public double IntakeMotorSpeed = 0;
    public double WristMotorSpeed = 0;
    public boolean CoralLimitSwitchState;
    public double EndEffectorAngleDegrees = 0;
    public double RangeSensorDistance = 0;
}
