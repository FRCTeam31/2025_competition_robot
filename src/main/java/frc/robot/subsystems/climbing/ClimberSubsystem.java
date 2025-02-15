package frc.robot.subsystems.climbing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ClimberSubsystem extends SubsystemBase {
    private IClimberIO Climber;
    private ClimberInputs m_inputs;

    private enum ClimberPosition {
        /** The starting position of the climber (Up) */
        IN,
        /** 𝓣𝓱𝓮 𝓮𝓷𝓭𝓲𝓷𝓰 𝓹𝓸𝓼𝓲𝓽𝓲𝓸𝓷 𝓸𝓯 𝓽𝓱𝓮 𝓬𝓵𝓲𝓶𝓫𝓮𝓻 (Down) */
        OUT
    }

    private ClimberPosition _climberPosition = ClimberPosition.IN;

    public ClimberSubsystem(Boolean isReal) {
        if (isReal) {
            Climber = new ClimberIOReal();
        } else {
            Climber = new ClimberIOSim();
        }

        m_inputs = Climber.getInputs();
    }

    @Override
    public void periodic() {
        m_inputs = Climber.getInputs();
        checkLimitSwitches();
    }

    private void checkLimitSwitches() {
        if (m_inputs.InLimitSwitch.get() || m_inputs.OutLimitSwitch.get()) {
            Climber.stopMotors();
        }
    }

    public Command setClimberPositionCommand(Value climberPosition) {
        return Commands.runOnce(() -> {
            Climber.setClimbersState(climberPosition);

        }, this);

    }

    public Command setClimberSpeedCommand(double speed) {
        return Commands.runOnce(() -> {
            Climber.setMotorSpeed(speed);
        }, this);

    }

    public Command toggleClimbersCommand() {
        return Commands.runOnce(() -> {
            if (m_inputs.OutLimitSwitch.get() || _climberPosition == ClimberPosition.OUT) {
                setClimberSpeedCommand(ClimberMap.climberInSpeed);
                _climberPosition = ClimberPosition.IN;
            } else if (m_inputs.InLimitSwitch.get() || _climberPosition == ClimberPosition.IN) {
                setClimberSpeedCommand(ClimberMap.climberOutSpeed);
                _climberPosition = ClimberPosition.OUT;
            }
        }, this);
    }
}