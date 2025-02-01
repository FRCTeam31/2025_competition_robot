package frc.robot.subsystems.climbing;

import java.util.Map;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ClimberSubsystem extends SubsystemBase {
    private IClimberIO Climber;

    public ClimberSubsystem(Boolean isReal) {
        if (isReal) {
            Climber = new ClimberIOReal();
        } else {
            Climber = new ClimberIOSim();
        }
    }

    public Command setClimberPosition(Value climberPosition) {
        return Commands.runOnce(() -> {
            Climber.setClimbersState(climberPosition);

        }, this);

    }

    public Command setClimberSpeed(double speed) {
        return Commands.runOnce(() -> {
            Climber.setMotorSpeed(speed);
        }, this);

    }

    public Map<String, Command> getNamedCommands() {
        return Map.of("Stop Climbers", setClimberSpeed(0), "Set Climber Position",
                setClimberPosition(Value.kForward));
    }
}