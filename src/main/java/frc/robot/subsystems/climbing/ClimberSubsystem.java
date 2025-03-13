package frc.robot.subsystems.climbing;

import java.util.function.BooleanSupplier;

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
        public static final double SetClimberStateSpeed = 0.8;
        public static final int ClimberOutLimitSwitchChannel = 5;
        public static final int ClimberInLimitSwitchChannel = 6;
        public static final double MaxMotorPercentOutput = 1;
        public static final double ActuallyClimbingSpeed = 0.8;
        public static final double MaxChangeClimberStateTime = 7;
        public static final int ClimberHookMotorCANID = 21;
        public static final int HooksOpenLimitSwitchChannel = 4;
        public static final int HooksClosedLimitSwitchChannel = 2;
        public static final double HooksOpenSpeed = -0.75;
        public static final double HooksCloseSpeed = 0.75;
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
        System.out.println("Set Hook State: " + hooksPosition.toString());
        double hooksMotorSpeed = 0;
        // boolean allowedToChangeHooksState = _inputs.ClimbWenchOutLimitSwitch
        //         && _inputs.RobotPitch > ClimberMap.ClimbingPitchThresholdDegrees;

        _inputs.CommandedHooksPosition = hooksPosition;
        // Only Allow the hooks to change state if the climber is  out
        // if (allowedToChangeHooksState) {
        if (hooksPosition == HooksPosition.CLOSED) {
            System.out.println("Hook will be closed");
            if (!_inputs.HooksClosedLimitSwitch) {
                System.out.println("Hooks Closed Limit Switch is false");
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
    public Command toggleHooksStateCommand() {
        HooksPosition commandedHooksState = (_inputs.CommandedHooksPosition == HooksPosition.CLOSED)
                ? HooksPosition.OPEN
                : HooksPosition.CLOSED;

        return _inputs.CommandedClimberPosition == ClimberPosition.OUT
                ? setHooksStateCommand(commandedHooksState)
                : Commands.none();
    }

    // public Command defaultClimbingCommand(BooleanSupplier climbIn, BooleanSupplier climbOut,
    //         BooleanSupplier hooksClosed, BooleanSupplier hooksOpen) {
    //     if (climbIn.getAsBoolean()) {

    //         return setClimberInCommand();
    //     } else if (climbOut.getAsBoolean()) {
    //         return setCLimberOutCommand();
    //     } else if (hooksClosed.getAsBoolean()
    //             && (_inputs.CommandedClimberPosition == ClimberPosition.OUT && _inputs.ClimbWenchOutLimitSwitch)) {
    //         return setHooksStateCommand(HooksPosition.CLOSED);
    //     } else if (hooksOpen.getAsBoolean()
    //             && (_inputs.CommandedClimberPosition == ClimberPosition.OUT && _inputs.ClimbWenchOutLimitSwitch)) {
    //         return setHooksStateCommand(HooksPosition.OPEN);
    //     } else {
    //         return new InstantCommand();
    //     }
    // }

    public Command setHooksStateCommand(HooksPosition hooksPosition) {
        return Commands.run(() -> setHooksState(hooksPosition))
                .until(() -> hooksPosition == HooksPosition.CLOSED
                        ? _inputs.HooksClosedLimitSwitch
                        : _inputs.HooksOpenLimitSwitch)
                .withTimeout(ClimberMap.MaxChangeHookStateTime)
                .andThen(stopHooksMotorsCommand());
    }

    public Command setClimberStateCommand(ClimberPosition climberPosition) {
        return Commands.run(() -> setClimberState(climberPosition))
                .until(() -> climberPosition == ClimberPosition.IN
                        ? _inputs.ClimbWenchInLimitSwitch
                        : _inputs.ClimbWenchOutLimitSwitch)
                .withTimeout(ClimberMap.MaxChangeClimberStateTime)
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
        // Only allowed to climb if the hooks are closed and the climber is in the out position
        return Commands.run(() -> {
            if (_inputs.ClimbWenchOutLimitSwitch && _inputs.CommandedHooksPosition == HooksPosition.CLOSED) {
                _climber.setClimbingWenchSpeed(ClimberMap.ActuallyClimbingSpeed);
            }
        });
    }

    public Command climbDownCommand() {
        // Only allowed to climb if the hooks are closed and the climber is in the out position
        return Commands.run(() -> {
            if (_inputs.ClimbWenchOutLimitSwitch && _inputs.CommandedHooksPosition == HooksPosition.CLOSED) {
                _climber.setClimbingWenchSpeed(-ClimberMap.ActuallyClimbingSpeed);
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