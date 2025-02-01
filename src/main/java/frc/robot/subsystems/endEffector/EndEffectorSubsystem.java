package frc.robot.subsystems.endEffector;

import frc.robot.subsystems.endEffector.IEndEffectorIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

    private IEndEffectorIO _endEffector;
    private EndEffectorIOInputs _inputs;

    public EndEffectorSubsystem(boolean isReal) {
        _endEffector = isReal ? new EndEffectorIOReal() : new EndEffectorIOSim();
    }

    public Command runEndEffectorCommand(double speed) {
        return this.run(() -> {
            _endEffector.setMotorSpeed(speed);
        });
    }

}