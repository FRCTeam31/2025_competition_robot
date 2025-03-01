package frc.robot.subsystems.algaeIntake;

import java.util.Map;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {

    public static class AlgaeIntakeMap {
        public static final byte algaeIntakeMotorCANID = 32;
        public static final byte leftAlgaeSolenoidForwardChannel = 0;
        public static final byte leftAlgaeSolenoidReverseChannel = 1;
        public static final byte rightAlgaeSolenoidForwardChannel = 2;
        public static final byte rightAlgaeSolenoidReverseChannel = 3;
        public static final byte algaeIntakeGearRatio = 1;
        public static final double AlgaeIntakeSpeed = 0.5;
        public static final double AlgaeEjectSpeed = -0.5;
    }

    private IAlgaeIntake _algaeIntake;

    private AlgaeIntakeInputsAutoLogged _inputs;

    public AlgaeIntakeSubsystem(boolean isReal) {
        _algaeIntake = isReal ? new AlgaeIntakeReal() : new AlgaeIntakeSim();

    }

    public void periodic() {
        _algaeIntake.updateInputs(_inputs);
    }

    public Command setAlgaeIntakePositionCommand(Value algaeIntakePosition) {
        return this.run(() -> {
            _algaeIntake.setAlgaeIntakePosition(algaeIntakePosition);
        });
    }

    public Command setAlgaeMotorSpeedCommand(double speed) {
        return this.run(() -> {
            _algaeIntake.setAlgaeMotorSpeed(speed);
        });
    }

    public Command stopAlgaeMotorCommand() {
        return this.run(() -> {
            _algaeIntake.stopMotors();
        });
    }

    public Map<String, Command> getNamedCommands() {
        return Map.of("Intake Algae", setAlgaeMotorSpeedCommand(AlgaeIntakeMap.AlgaeIntakeSpeed), "Eject Algae",
                setAlgaeMotorSpeedCommand(AlgaeIntakeMap.AlgaeEjectSpeed), "Stop Algae Motor", stopAlgaeMotorCommand(),
                "Extend Algae Intake", setAlgaeIntakePositionCommand(Value.kForward), "Retract Algae Intake",
                setAlgaeIntakePositionCommand(Value.kReverse));
    }
}