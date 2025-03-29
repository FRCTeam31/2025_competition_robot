package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimberInputs {
    public double WinchMotorSpeed = 0;
    public double HooksMotorSpeed = 0;

    // Limit Switches. If the boolean is true the limit switch is pressed if it is false the limit switch is not pressed
    public boolean WinchInnerLimitSwitch;
    public boolean WinchOuterLimitSwitch;
    public boolean HooksOpenLimitSwitch;
    public boolean HooksClosedLimitSwitch;
    public double climberShaftRotations;
}
