package frc.robot.subsystems.climbing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climbing.ClimberInputs.ClimberPosition;
import frc.robot.subsystems.climbing.ClimberInputs.ServoPosition;

@Logged
public class ClimberSubsystem extends SubsystemBase {
    private IClimberIO _climber;
    private ClimberInputs _inputs;

    public ClimberSubsystem(Boolean isReal) {
        if (isReal) {
            _climber = new ClimberIOReal();
        } else {
            _climber = new ClimberIOSim();
        }

        _inputs = _climber.updateInputs();
    }

    @Override
    public void periodic() {
        _inputs = _climber.updateInputs();
    }

    public Command stopClimbingMotorsCommand() {
        return Commands.runOnce(() -> {
            _climber.stopMotors();
        }, this);
    }

    public Command setHooksStateCommand(ServoPosition servoPosition) {
        return Commands.runOnce(() -> {
            // Only Allow the hooks to change state if the climber is  out
            if (_inputs.OutLimitSwitch) {
                _climber.setHooksState(servoPosition);
            }

        }, this);

    }

    public Command toggleHooksStateCommand() {
        return Commands.runOnce(() -> {
            if (_inputs.OutLimitSwitch) {
                // Toggle the commanded hooks state
                ServoPosition commandedHooksState = (_inputs.CommandedServoPosition == ServoPosition.CLOSED)
                        ? ServoPosition.OPEN
                        : ServoPosition.CLOSED;

                _climber.setHooksState(commandedHooksState);
                _inputs.CommandedServoPosition = commandedHooksState;
            }

        }, this);

    }

    public void setClimberState(ClimberPosition climberPosition) {
        double motorSpeed = 0;
        _inputs.CommandedClimberPosition = climberPosition;

        if (climberPosition == ClimberPosition.IN) {
            if (!_inputs.InLimitSwitch) {
                motorSpeed = ClimberMap.ClimberInSpeed;
            } else {
                motorSpeed = 0; // Stop motor if InLimitSwitch is pressed
            }
        } else if (climberPosition == ClimberPosition.OUT) {
            if (!_inputs.OutLimitSwitch) {
                motorSpeed = ClimberMap.ClimberOutSpeed;
            } else {
                motorSpeed = 0; // Stop motor if OutLimitSwitch is pressed
            }
        }
        _climber.setMotorSpeed(motorSpeed);
    }

    // Remember to ask mason about the command decorator thing
    public Command setClimberOutCommand() {
        return Commands.run(() -> {
            _climber.setHooksState(ServoPosition.CLOSED);
            _inputs.CommandedServoPosition = ServoPosition.CLOSED;

            setClimberState(ClimberPosition.OUT);
        }, this).until(() -> _inputs.OutLimitSwitch).withTimeout(5).andThen(stopClimbingMotorsCommand());
    }

    public Command setClimberInCommand() {
        return Commands.run(() -> {
            _climber.setHooksState(ServoPosition.OPEN);
            setClimberState(ClimberPosition.IN);
        }, this).until(() -> _inputs.InLimitSwitch).withTimeout(5).andThen(stopClimbingMotorsCommand());
    }

}