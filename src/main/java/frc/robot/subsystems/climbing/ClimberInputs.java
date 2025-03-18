package frc.robot.subsystems.climbing;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimberInputs {
    public enum ClimberPosition {
        /** The starting position of the climber (Up) */
        IN,
        /** The ending position of the climber (Down) */
        OUT
    }

    public enum HooksPosition {
        /** The servo arms  will be open */
        OPEN,
        /** The servo arms  will be closed) */
        CLOSED
    }

    public double ClimbWenchMotorSpeed = 0;
    public double HooksMotorSpeed = 0;
    // Limit Switches. If the boolean is true the limit switch is pressed if it is false the limit switch is not pressed
    public boolean ClimbWenchInLimitSwitch;
    public boolean ClimbWenchOutLimitSwitch;
    public boolean HooksOpenLimitSwitch;
    public boolean HooksClosedLimitSwitch;
    public double ClimberAngleDegrees;

}
