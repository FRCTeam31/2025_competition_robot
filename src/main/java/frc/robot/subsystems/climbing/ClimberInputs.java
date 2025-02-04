package frc.robot.subsystems.climbing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@Logged
public class ClimberInputs {

    public double ClimbMotorSpeed = 0;
    public Value ClimbSolenoidPosition;
    public DigitalInput InLimitSwitch;
    public DigitalInput OutLimitSwitch;

    public ClimberInputs() {

    }

    public ClimberInputs(double climbMotorSpeed, Value climbSolenoidPosition, DigitalInput inLimitSwitch,
            DigitalInput outLimitSwitch) {
        ClimbMotorSpeed = climbMotorSpeed;
        ClimbSolenoidPosition = climbSolenoidPosition;
        InLimitSwitch = inLimitSwitch;
        OutLimitSwitch = outLimitSwitch;
    }
}
