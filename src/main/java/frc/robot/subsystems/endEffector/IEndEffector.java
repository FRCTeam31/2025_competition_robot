package frc.robot.subsystems.endEffector;

public interface IEndEffector {

    public void updateInputs(EndEffectorInputsAutoLogged inputs);

    public void setIntakeMotorSpeed(double speed);

    public void setWristMotorSpeed(double speed);

    public void stopIntakeMotor();

    public void stopWristMotor();

    public void stopMotors();
}