package frc.robot.subsystems.climber;

public interface IClimber {

    public void updateInputs(ClimberInputsAutoLogged inputs);

    public void setWinchSpeed(double Speed);

    public void setHookMotorSpeed(double Speed);

    public void stopWinchMotors();

    public void stopHooksMotors();

    public void resetClimberAngle();
}
