package frc.robot.subsystems.climbing;

public interface IClimberIO {

    public void updateInputs(ClimberInputsAutoLogged inputs);

    public void setClimbingWenchSpeed(double Speed);

    public void setHookMotorSpeed(double Speed);

    public void stopWenchMotors();

    public void stopHooksMotors();

}
