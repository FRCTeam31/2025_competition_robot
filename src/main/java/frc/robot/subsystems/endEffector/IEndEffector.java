package frc.robot.subsystems.endEffector;

public interface IEndEffector {

    public void updateInputs(EndEffectorInputsAutoLogged inputs);

    public void setMotorSpeed(double speed);

    public void stopMotors();
}