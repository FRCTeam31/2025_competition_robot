package frc.robot.subsystems.endEffector;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

    private IEndEffector _endEffector;
    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    public EndEffectorSubsystem(boolean isReal) {
        _endEffector = isReal ? new EndEffectorReal() : new EndEffectorSim();
    }

    public Command runEndEffectorCommand(double speed) {
        return this.run(() -> {
            _endEffector.setMotorSpeed(speed);
        });
    }

    public Command stopEndEffectorCommand() {
        return this.run(() -> {
            _endEffector.stopMotors();
        });
    }

    public Map<String, Command> getNamedCommands() {
        return Map.of("Stop Intake", stopEndEffectorCommand(), "Intake Coral",
                runEndEffectorCommand(EndEffectorMap.IntakeSpeed), "Eject Coral",
                runEndEffectorCommand(EndEffectorMap.EjectSpeed));
    }

}