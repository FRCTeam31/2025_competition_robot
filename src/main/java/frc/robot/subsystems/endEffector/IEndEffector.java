package frc.robot.subsystems.endEffector;

public interface IEndEffector {

    public void updateInputs(EndEffectorInputsAutoLogged inputs);

    public void setIntakeSpeed(double speed);

    public void setWristSpeed(double speed);

    public void stopIntakeMotor();

    public void stopWristMotor();

    public void stopMotors();

}