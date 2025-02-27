package frc.robot.subsystems.climbing;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.climbing.ClimberInputs.ServoPosition;

public interface IClimberIO {

    public ClimberInputs updateInputs();

    public void setMotorSpeed(double Speed);

    public void stopMotors();

    public void setHooksState(ServoPosition hooksCommmandedPosition);

}
