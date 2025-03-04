package frc.robot.subsystems.endEffector;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

    private IEndEffector _endEffector;

    private PIDController _wristPidController = new PIDController(EndEffectorMap.WristPID.kP,
            EndEffectorMap.WristPID.kI,
            EndEffectorMap.WristPID.kD);

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

    public Command runEndEffectorIntakeCommand(double speed) {
        return this.run(() -> {
            _endEffector.setIntakeMotorSpeed(speed);
        });
    }

    public Command stopEndEffectorIntakeCommand() {
        return this.run(() -> {
            _endEffector.stopIntakeMotor();
        });
    }

    public Map<String, Command> getNamedCommands() {
        return Map.of("Stop Intake", stopEndEffectorIntakeCommand(), "Intake Coral",
                runEndEffectorIntakeCommand(EndEffectorMap.IntakeSpeed), "Eject Coral",
                runEndEffectorIntakeCommand(EndEffectorMap.EjectSpeed));
    }

}