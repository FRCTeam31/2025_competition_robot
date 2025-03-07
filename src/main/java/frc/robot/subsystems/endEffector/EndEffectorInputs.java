package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class EndEffectorInputs {
    public double IntakeMotorSpeed = 0;
    public double WristMotorSpeed = 0;
    public boolean LimitSwitchState;
    public double EndEffectorAngleDegrees = 0;
}
