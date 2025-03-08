package frc.robot.subsystems.endEffector;

import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IEndEffector {

    public void updateInputs(EndEffectorInputsAutoLogged inputs);

    public void setIntakeSpeed(double speed);

    public void setWristSpeed(double speed);

    public void stopIntakeMotor();

    public void stopWristMotor();

    public void stopMotors();

}