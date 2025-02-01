package frc.robot.subsystems.climbing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@Logged
public class ClimberInputs {

    public double ClimbMotorSpeed = 0;
    public Value ClimbSolenoidPosition;

    public ClimberInputs() {

    }

    public ClimberInputs(double climbMotorSpeed, Value climbSolenoidPosition) {
        ClimbMotorSpeed = climbMotorSpeed;
        ClimbSolenoidPosition = climbSolenoidPosition;

    }
}
