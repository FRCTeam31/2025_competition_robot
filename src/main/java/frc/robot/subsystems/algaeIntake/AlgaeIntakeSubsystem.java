package frc.robot.subsystems.algaeIntake;

import java.util.Map;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.AlgaeIntakeMap;

public class AlgaeIntakeSubsystem extends SubsystemBase {

    private IAlgaeIntakeIO AlgaeIntake;

    public AlgaeIntakeSubsystem(boolean isReal) {
        if (isReal) {
            AlgaeIntake = new AlgaeIntakeIOReal();
        } else {
            AlgaeIntake = new AlgaeIntakeIOSim();
        }
    }

    public Command setAlgaeIntakePositionCommand(Value algaeIntakePosition) {
        return this.run(() -> {
            AlgaeIntake.setAlgaeIntakePosition(algaeIntakePosition);
        });
    }

    public Command setAlgaeMotorSpeedCommand(double speed) {
        return this.run(() -> {
            AlgaeIntake.setAlgaeMotorSpeed(speed);
        });
    }

    public Command stopAlgaeMotorCommand() {
        return this.run(() -> {
            AlgaeIntake.stopMotors();
        });
    }

    public Map<String, Command> getNamedCommands() {
        return Map.of("Intake Algae", setAlgaeMotorSpeedCommand(AlgaeIntakeMap.AlgaeIntakeSpeed), "Eject Algae",
                setAlgaeMotorSpeedCommand(AlgaeIntakeMap.AlgaeEjectSpeed), "Stop Algae Motor", stopAlgaeMotorCommand(),
                "Extend Algae Intake", setAlgaeIntakePositionCommand(Value.kForward), "Retract Algae Intake",
                setAlgaeIntakePositionCommand(Value.kReverse));
    }
}