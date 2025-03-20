package frc.robot.subsystems.climbing;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Container;
import frc.robot.Robot;
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
        public static final double ClimberSpeed = 1;
        public static final double HooksSpeed = 1;
        public static final double ClimberChangeStateTimeout = 7;
        public static final double HooksChangeStateTimeout = 5;
        public static final double ClimbingPitchThresholdDegrees = 5;
        public static final double fullyClimbedOutputRotations = 3;
        public static final double ClimbAngleResetDebounceSeconds = 0.25;
        public static final double ClimberArmlengthMeters = Units.inchesToMeters(16);
        public static final double ClimberOutPutShaftDiamateterMeters = Units.inchesToMeters(0.5);

    }

    private IClimberIO _climber;
    private ClimberInputsAutoLogged _inputs = new ClimberInputsAutoLogged();
    BooleanEvent _positionResetEvent;
    Trigger _setClimberOutTimedTrigger;
    boolean allowedToOpenHooks = false;

    // Ask mason how he wants you to update this using the gyro
    private double _robotPitch = 0;

    public ClimberSubsystem(Boolean isReal) {
        setName("Climber");
        if (isReal) {
            _climber = new ClimberIOReal();

        } else {
            _climber = new ClimberIOSim();
        }

        _positionResetEvent = new BooleanEvent(Robot.EventLoop, () -> _inputs.ClimbWenchOutLimitSwitch)
                .debounce(ElevatorMap.BottomLimitResetDebounceSeconds)
                .rising();

        _positionResetEvent.ifHigh(() -> {
            _climber.resetClimberAngle();

        });

        _setClimberOutTimedTrigger = new Trigger(Robot.EventLoop,
                () -> DriverStation.getMatchTime() <= 50 && DriverStation.isTeleopEnabled());

        _setClimberOutTimedTrigger.onTrue(
                setHooksClosedAuto().asProxy()
                        .alongWith(setClimberOutAuto().asProxy()));

    }

    @Override
    public void periodic() {
        _climber.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
        SmartDashboard.putBoolean(getName() + "allowed to change hooks state", allowedToOpenHooks);
        Logger.recordOutput(getName() + "/climberAngleDegrees", _inputs.climberShaftRotations);

    }

    private void setHooksSpeed(double speed) {
        // Positive is closed, negative is open
        boolean currentlyClimbing = _robotPitch >= ClimberMap.ClimbingPitchThresholdDegrees;
        if (!currentlyClimbing) {
            if (speed > 0 && !_inputs.HooksClosedLimitSwitch) {
                _climber.setHookMotorSpeed(speed);
            } else if (speed < 0 && !_inputs.HooksOpenLimitSwitch) {
                _climber.setHookMotorSpeed(speed);
            } else {
                _climber.stopHooksMotors();
            }
        }
        SmartDashboard.putNumber(getName() + "/HooksSpeed", speed);
    }

    private void setClimberSpeed(double speed) {
        // in is positive, out is negative
        if (speed > 0 && !_inputs.ClimbWenchInLimitSwitch) {
            if (_inputs.climberShaftRotations > ClimberMap.fullyClimbedOutputRotations) {
                _climber.stopWenchMotors();
            } else {
                _climber.setClimbingWenchSpeed(speed);
            }
        } else if (speed < 0 && !_inputs.ClimbWenchOutLimitSwitch) {
            _climber.setClimbingWenchSpeed(speed);
        } else {
            _climber.stopWenchMotors();
        }

        SmartDashboard.putNumber(getName() + "/ClimberSpeed", speed);

    }

    //#region Commands

    public Command setClimberOutCommand() {
        return this.run(() -> setClimberSpeed(-ClimberMap.ClimberSpeed));
    }

    public Command setClimberInCommand() {
        return this.run(() -> setClimberSpeed(ClimberMap.ClimberSpeed));
    }

    public Command setHooksClosedCommand() {
        return this.run(() -> setHooksSpeed(ClimberMap.HooksSpeed));
    }

    public Command setHooksOpenCommand() {
        return this.run(() -> setHooksSpeed(-ClimberMap.HooksSpeed));
    }

    public Command setHooksClosedAuto() {
        return setHooksClosedCommand()
                .until(() -> _inputs.HooksClosedLimitSwitch)
                .andThen(stopHooksMotorsCommand());
    }

    public Command setClimberOutAuto() {
        return setClimberOutCommand()
                .until(() -> _inputs.ClimbWenchOutLimitSwitch)
                .andThen(stopClimbingMotorsCommand());
    }

    public Command climbOffTheGround() {
        return setClimberInCommand().until(() -> _inputs.climberShaftRotations > ClimberMap.fullyClimbedOutputRotations)
                .withTimeout(5).andThen(stopClimbingMotorsCommand());
    }

    public Command fullyClimbInManual() {
        return this.run(() -> {
            if (!_inputs.ClimbWenchInLimitSwitch) {
                _climber.setClimbingWenchSpeed(ClimberMap.ClimberSpeed);
            } else {
                _climber.stopWenchMotors();
            }
        });
    }

    public Command stopClimbingMotorsCommand() {
        return this.runOnce(_climber::stopWenchMotors);
    }

    public Command stopHooksMotorsCommand() {
        return this.runOnce(_climber::stopHooksMotors);
    }

    public Command stopAllMotors() {
        return stopClimbingMotorsCommand().andThen(stopHooksMotorsCommand());
    }

    //#endregion
}