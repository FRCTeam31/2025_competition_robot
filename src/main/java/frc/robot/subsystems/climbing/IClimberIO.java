package frc.robot.subsystems.climbing;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.climbing.ClimberInputs.HooksPosition;

public interface IClimberIO {

    public ClimberInputs updateInputs();

    public void setClimbingWenchSpeed(double Speed);

    public void setHookMotorSpeed(double Speed);

    public void stopWenchMotors();

    public void stopHooksMotors();

}
