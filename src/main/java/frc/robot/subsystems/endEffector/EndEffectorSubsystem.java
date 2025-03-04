package frc.robot.subsystems.endEffector;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

    private IEndEffector _endEffector;
    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    public EndEffectorSubsystem(boolean isReal) {
        setName("End Effector");
        _endEffector = isReal ? new EndEffectorReal() : new EndEffectorSim();
    }

    @Override
    public void periodic() {
        _endEffector.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
    }

    public Command runEndEffectorCommand(double speed) {
        return this.run(() -> {
            _endEffector.setIntakeMotorSpeed(speed);
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