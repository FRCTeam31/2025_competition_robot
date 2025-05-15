package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.prime.control.SubsystemControlMode;

@AutoLog
public class ElevatorInputs {

    // IO inputs
    public double MotorSpeed = 0;
    public double MotorVoltage = 0;
    public double DistanceMeters = 0;
    public double SpeedMPS = 0;
    public boolean TopLimitSwitch;
    public boolean BottomLimitSwitch;

    // Subsystem state
    public SubsystemControlMode ControlMode = SubsystemControlMode.ManuallyControlled;
}
