package frc.robot.subsystems.endEffector;

public interface IEndEffectorIO {

    public EndEffectorIOInputs getInputs();

    public void setMotorSpeed(double speed);

    public void stopMotors();
}