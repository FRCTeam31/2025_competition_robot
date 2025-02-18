package frc.robot.subsystems.climbing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@Logged
public class ClimberInputs {

    public double ClimbMotorSpeed = 0;
    public Value ClimbSolenoidPosition;
    public boolean InLimitSwitch;
    public boolean OutLimitSwitch;

    public ClimberInputs() {

    }

    public ClimberInputs(double climbMotorSpeed, Value climbSolenoidPosition, boolean inLimitSwitch,
            boolean outLimitSwitch) {
        ClimbMotorSpeed = climbMotorSpeed;
        ClimbSolenoidPosition = climbSolenoidPosition;
        InLimitSwitch = inLimitSwitch;
        OutLimitSwitch = outLimitSwitch;
    }
}
