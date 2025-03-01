package frc.robot.subsystems.climbing;

public interface IClimberIO {

    public ClimberInputs updateInputs();

    public void setClimbingWenchSpeed(double Speed);

    public void setHookMotorSpeed(double Speed);

    public void stopWenchMotors();

    public void stopHooksMotors();

}
