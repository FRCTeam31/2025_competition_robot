package frc.robot.subsystems.endEffector;

public class EndEffectorIOInputs {
    double VelocityRadPerSec = 0;

    public void getInputs(double velocityRadPerSec) {
        VelocityRadPerSec = velocityRadPerSec;
    }
}
