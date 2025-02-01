package frc.robot.subsystems.endEffector;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.struct.StructSerializable;

@Logged
public class EndEffectorIOOutputs implements StructSerializable {

    public EndEffectorIOOutputs(double speed) {
        Speed = speed;
    }

    public double Speed = 0;
}
