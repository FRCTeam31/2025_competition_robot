package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;
import org.prime.control.SubsystemControlMode;

@AutoLog
public class EndEffectorInputs {
    // IO inputs
    public double WristMotorSpeed = 0;
    public double EndEffectorAngleDegrees = 0;

    public double IntakeMotorSpeed = 0;
    public double RangeSensorDistance = 0;
    public boolean CoralLimitSwitchState;

    // Subsystem State
    public SubsystemControlMode ControlMode = SubsystemControlMode.ClosedLoopControlled;
    public double ManualControlSpeed = 0;
    public boolean ManuallyEjecting = false;
}
