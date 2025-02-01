package frc.robot.subsystems.endEffector;

public class EndEffectorIOOutputs implements IEndEffectorIO {

    public EndEffectorIOOutputs(double speed) {
        Speed = speed;
    }

    public double Speed = 0;
}
