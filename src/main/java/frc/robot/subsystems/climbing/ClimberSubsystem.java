package frc.robot.subsystems.climbing;

import java.util.Map;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.ClimberMap;

@Logged
public class ClimberSubsystem extends SubsystemBase {
    private IClimberIO Climber;
    /**
     * This is the climber inputs
     * It is the inputs variable
     */
    private ClimberInputs _inputs = new ClimberInputs();

    private enum ClimberPosition {
        /** The starting position of the climber (Up) */
        IN,
        /** 𝓣𝓱𝓮 𝓮𝓷𝓭𝓲𝓷𝓰 𝓹𝓸𝓼𝓲𝓽𝓲𝓸𝓷 𝓸𝓯 𝓽𝓱𝓮 𝓬𝓵𝓲𝓶𝓫𝓮𝓻 */
        OUT
    }

    private ClimberPosition _climberPosition = ClimberPosition.IN;

    public ClimberSubsystem(Boolean isReal) {
        if (isReal) {
            Climber = new ClimberIOReal();
        } else {
            Climber = new ClimberIOSim();
        }
    }

    @Override
    public void periodic() {
        _inputs = Climber.getInputs();
        checkLimitSwitches();
    }

    public void checkLimitSwitches() {
        if (_inputs.InLimitSwitch.get() || _inputs.OutLimitSwitch.get()) {
            Climber.stopMotors();
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

    public Command toggleClimbers() {
        return Commands.runOnce(() -> {
            if (_inputs.OutLimitSwitch.get() || _climberPosition == ClimberPosition.OUT) {
                setClimberSpeed(ClimberMap.climberInSpeed);
                _climberPosition = ClimberPosition.IN;
            } else if (_inputs.InLimitSwitch.get() || _climberPosition == ClimberPosition.IN) {
                setClimberSpeed(ClimberMap.climberOutSpeed);
                _climberPosition = ClimberPosition.OUT;
            }
        }, this);
    }

    public Map<String, Command> getNamedCommands() {
        return Map.of("Stop Climbers", setClimberSpeed(0), "Set Climber Position",
                setClimberPosition(Value.kForward));
    }
}