package frc.robot.subsystems.climbing;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climbing.ClimberInputs.ClimberPosition;
import frc.robot.subsystems.climbing.ClimberInputs.HooksPosition;

public class ClimberSubsystem extends SubsystemBase {

    public static class ClimberMap {
        public static final byte ClimberLeftMotorCANID = 17;
        public static final byte ClimberRightMotorCANID = 18;
        public static final byte ClimberGearRatio = 100;
        public static final double ClimberInSpeed = 0.5;
        public static final double ClimberOutSpeed = -0.5;
        public static final int ClimberOutLimitSwitchChannel = 5;
        public static final int ClimberInLimitSwitchChannel = 6;
        public static final double MaxMotorPercentOutput = 1;
        public static final double ClimberMotorsReelingInSpeed = 0.5;
        public static final double ClimberMotorsReelingOutSpeed = -0.5;
        public static final double MaxChangeClimberStateTime = 5;
        public static final int ClimberHookMotorCANID = 21;
        public static final int HooksOpenLimitSwitchChannel = 4;
        public static final int HooksClosedLimitSwitchChannel = 2;
        public static final double HooksOpenSpeed = 0.5;
        public static final double HooksCloseSpeed = -0.5;
        public static final double MaxChangeHookStateTime = 5;
        public static final double ClimbingPitchThresholdDegrees = 5;

    }

    private IClimberIO _climber;
    private ClimberInputsAutoLogged _inputs = new ClimberInputsAutoLogged();

    public ClimberSubsystem(Boolean isReal) {
        setName("Climber");
        if (isReal) {
            _climber = new ClimberIOReal();

        } else {
            _climber = new ClimberIOSim();

        }
    }

    @Override
    public void periodic() {
        _climber.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);

    }

    private void setHooksState(HooksPosition hooksPosition) {
        double hooksMotorSpeed = 0;
        boolean allowedToChangeHooksState = _inputs.ClimbWenchOutLimitSwitch
                && _inputs.RobotPitch > ClimberMap.ClimbingPitchThresholdDegrees;

        _inputs.CommandedHooksPosition = hooksPosition;
        // Only Allow the hooks to change state if the climber is  out
        if (allowedToChangeHooksState) {
            if (hooksPosition == HooksPosition.CLOSED) {
                if (!_inputs.HooksClosedLimitSwitch) {
                    hooksMotorSpeed = ClimberMap.HooksCloseSpeed;
                } else {
                    hooksMotorSpeed = 0; // Stop motor if InLimitSwitch is pressed
                }
            } else if (hooksPosition == HooksPosition.OPEN) {
                if (!_inputs.HooksOpenLimitSwitch) {
                    hooksMotorSpeed = ClimberMap.HooksOpenSpeed;
                } else {
                    hooksMotorSpeed = 0; // Stop motor if OutLimitSwitch is pressed
                }
            }

        }
        _climber.setHookMotorSpeed(hooksMotorSpeed);

    }

    private void setClimberState(ClimberPosition climberPosition) {
        double wenchSpeed = 0;
        _inputs.CommandedClimberPosition = climberPosition;

        if (climberPosition == ClimberPosition.IN) {
            if (!_inputs.ClimbWenchInLimitSwitch) {
                wenchSpeed = ClimberMap.ClimberInSpeed;
            } else {
                wenchSpeed = 0; // Stop motor if InLimitSwitch is pressed
            }
        } else if (climberPosition == ClimberPosition.OUT) {
            if (!_inputs.ClimbWenchOutLimitSwitch) {
                wenchSpeed = ClimberMap.ClimberOutSpeed;
            } else {
                wenchSpeed = 0; // Stop motor if OutLimitSwitch is pressed
            }
        }
        _climber.setClimbingWenchSpeed(wenchSpeed);
    }

    public Command toggleHooksStateCommand() {

        HooksPosition commandedHooksState = (_inputs.CommandedHooksPosition == HooksPosition.CLOSED)
                ? HooksPosition.OPEN
                : HooksPosition.CLOSED;

        if (_inputs.CommandedClimberPosition == ClimberPosition.OUT) {
            return setHooksStateCommand(commandedHooksState);
        } else {
            return new InstantCommand();

        }
    }

    public Command setHooksStateCommand(HooksPosition hooksPosition) {
        return this.run(() -> {
            setHooksState(hooksPosition);
        }).until(() -> hooksPosition == HooksPosition.CLOSED ? _inputs.HooksClosedLimitSwitch
                : _inputs.HooksOpenLimitSwitch).withTimeout(ClimberMap.MaxChangeHookStateTime)
                .andThen(stopHooksMotorsCommand());
    }

    public Command setClimberStateCommand(ClimberPosition climberPosition) {
        return this.run(() -> {
            setClimberState(climberPosition);
        }).until(() -> climberPosition == ClimberPosition.IN ? _inputs.ClimbWenchInLimitSwitch
                : _inputs.ClimbWenchOutLimitSwitch).withTimeout(ClimberMap.MaxChangeClimberStateTime)
                .andThen(stopClimbingMotorsCommand());
    }

    public Command setCLimberOutCommand() {
        return setClimberStateCommand(ClimberPosition.OUT);
    }

    public ParallelCommandGroup setClimberInCommand() {
        return new ParallelCommandGroup(setHooksStateCommand(HooksPosition.CLOSED),
                setClimberStateCommand(ClimberPosition.IN));
    }

    public Command climbUpCommand() {
        // Only allowed to climb if the hooks are closed and the climber is in the out position
        return this.run(() -> {
            if (_inputs.ClimbWenchOutLimitSwitch && _inputs.CommandedHooksPosition == HooksPosition.CLOSED) {
                _climber.setClimbingWenchSpeed(ClimberMap.ClimberMotorsReelingInSpeed);
            }
        });
    }

    public Command climbDownCommand() {
        // Only allowed to climb if the hooks are closed and the climber is in the out position
        return Commands.run(() -> {
            if (_inputs.ClimbWenchOutLimitSwitch && _inputs.CommandedHooksPosition == HooksPosition.CLOSED) {
                _climber.setClimbingWenchSpeed(ClimberMap.ClimberMotorsReelingOutSpeed);
            }
        });
    }

    public Command climbManuallyCommand(double speed) {
        return this.run(() -> {
            _climber.setClimbingWenchSpeed(speed);
        }).until(() -> _inputs.ClimbWenchInLimitSwitch || _inputs.ClimbWenchOutLimitSwitch)
                .andThen(stopClimbingMotorsCommand());
    }

    public Command setHooksManual(double speed) {
        return this.run(() -> {
            _climber.setHookMotorSpeed(speed);
        });
    }

    public Command stopClimbingMotorsCommand() {
        return Commands.runOnce(() -> {
            _climber.stopWenchMotors();
        }, this);
    }

    public Command stopHooksMotorsCommand() {
        return Commands.runOnce(() -> {
            _climber.stopHooksMotors();
        }, this);
    }
}