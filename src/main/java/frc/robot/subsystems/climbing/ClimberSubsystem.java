package frc.robot.subsystems.climbing;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Container;
import frc.robot.subsystems.climbing.ClimberInputs.ClimberPosition;
import frc.robot.subsystems.climbing.ClimberInputs.HooksPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMap;

public class ClimberSubsystem extends SubsystemBase {

    public static class ClimberMap {
        public static final byte ClimberGearRatio = 100;
        public static final byte ClimberLeftMotorCANID = 17;
        public static final byte ClimberRightMotorCANID = 18;
        public static final byte ClimberHookMotorCANID = 21;
        public static final byte HooksClosedLimitSwitchChannel = 2;
        public static final byte HooksOpenLimitSwitchChannel = 4;
        public static final byte ClimberOutLimitSwitchChannel = 5;
        public static final byte ClimberInLimitSwitchChannel = 6;
        public static final double SetClimberStateSpeed = 0.8;
        public static final double ActuallyClimbingSpeed = 0.8;
        public static final double HooksSpeed = 0.75;
        public static final double ClimberChangeStateTimeout = 7;
        public static final double HooksChangeStateTimeout = 5;
        public static final double ClimbingPitchThresholdDegrees = 5;

    }

    private IClimberIO _climber;
    private ClimberInputsAutoLogged _inputs = new ClimberInputsAutoLogged();

    // Ask mason how he wants you to update this using the gyro
    private double _robotPitch = 0;

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
        boolean currentlyClimbing = _robotPitch >= ClimberMap.ClimbingPitchThresholdDegrees;
        boolean allowedToOpenHooks = _inputs.ClimbWenchOutLimitSwitch;
        _inputs.CommandedHooksPosition = hooksPosition;

        if (hooksPosition == HooksPosition.CLOSED) {
            if (!_inputs.HooksClosedLimitSwitch && (!currentlyClimbing)) {
                hooksMotorSpeed = ClimberMap.HooksSpeed;
            } else {
                hooksMotorSpeed = 0; // Stop motor if InLimitSwitch is pressed
            }
        } else if (hooksPosition == HooksPosition.OPEN ) {
            if (!_inputs.HooksOpenLimitSwitch && (!currentlyClimbing && allowedToOpenHooks)) {
                hooksMotorSpeed = -ClimberMap.HooksSpeed;
            } else {
                hooksMotorSpeed = 0; // Stop motor if OutLimitSwitch is pressed
            }
        } 

        Logger.recordOutput(getName() + "/HooksSpeed", hooksMotorSpeed);
        _climber.setHookMotorSpeed(hooksMotorSpeed);
    }

    private void setClimberState(ClimberPosition climberPosition) {
        double wenchSpeed = 0;
        _inputs.CommandedClimberPosition = climberPosition;

        if (climberPosition == ClimberPosition.IN) {
            if (!_inputs.ClimbWenchInLimitSwitch) {
                wenchSpeed = ClimberMap.SetClimberStateSpeed;
            } else {
                wenchSpeed = 0; // Stop motor if InLimitSwitch is pressed
            }
        } else if (climberPosition == ClimberPosition.OUT) {
            if (!_inputs.ClimbWenchOutLimitSwitch) {
                wenchSpeed = -ClimberMap.SetClimberStateSpeed;
            } else {
                wenchSpeed = 0; // Stop motor if OutLimitSwitch is pressed
            }
        }
        _climber.setClimbingWenchSpeed(wenchSpeed);
    }

    //#region Commands

    public Command setHooksStateCommand(HooksPosition hooksPosition) {
        return Commands.run(() -> setHooksState(hooksPosition))
                .until(() -> hooksPosition == HooksPosition.CLOSED
                        ? _inputs.HooksClosedLimitSwitch
                        : _inputs.HooksOpenLimitSwitch)
                .withTimeout(ClimberMap.HooksChangeStateTimeout)
                .andThen(stopHooksMotorsCommand());
    }

    public Command setClimberStateCommand(ClimberPosition climberPosition) {
        return Commands.run(() -> setClimberState(climberPosition))
                .until(() -> climberPosition == ClimberPosition.IN
                        ? _inputs.ClimbWenchInLimitSwitch
                        : _inputs.ClimbWenchOutLimitSwitch)
                .withTimeout(ClimberMap.ClimberChangeStateTimeout)
                .andThen(stopClimbingMotorsCommand());
    }

    public Command setClimberOutCommand() {
        return setClimberStateCommand(ClimberPosition.OUT);
    }

    public ParallelCommandGroup setClimberInCommand() {
        return setHooksStateCommand(HooksPosition.CLOSED)
                .alongWith(setClimberStateCommand(ClimberPosition.IN));
    }

    public Command climbUpCommand() {
        // Only allowed to climb if the hooks are open
        return Commands.runOnce(() -> {
            if (_inputs.HooksOpenLimitSwitch && !_inputs.ClimbWenchInLimitSwitch) {
                _climber.setClimbingWenchSpeed(ClimberMap.ActuallyClimbingSpeed);
            } else if (_inputs.ClimbWenchInLimitSwitch) {
                stopClimbingMotorsCommand();
            }
        });
    }

    public Command climbDownCommand() {
        // Only allowed to climb if the hooks are open
        return Commands.runOnce(() -> {
            if (_inputs.HooksOpenLimitSwitch && !_inputs.ClimbWenchOutLimitSwitch) {
                _climber.setClimbingWenchSpeed(-ClimberMap.ActuallyClimbingSpeed);
            } else if (_inputs.ClimbWenchOutLimitSwitch) {
                stopClimbingMotorsCommand();
            }
        });
    }

    public Command stopClimbingMotorsCommand() {
        return Commands.runOnce(_climber::stopWenchMotors);
    }

    public Command stopHooksMotorsCommand() {
        return Commands.runOnce(_climber::stopHooksMotors);
    }

    //#endregion
}