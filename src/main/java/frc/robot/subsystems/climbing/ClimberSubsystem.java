package frc.robot.subsystems.climbing;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        public static final double ClimberSpeed = 0.8;
        public static final double HooksSpeed = 0.4;
        public static final double ClimberChangeStateTimeout = 7;
        public static final double HooksChangeStateTimeout = 5;
        public static final double ClimbingPitchThresholdDegrees = 5;

    }

    private IClimberIO _climber;
    private ClimberInputsAutoLogged _inputs = new ClimberInputsAutoLogged();
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
    }

    @Override
    public void periodic() {
        _climber.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
        SmartDashboard.putBoolean(getName() + "allowed to change hooks state", allowedToOpenHooks);

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
            _climber.setClimbingWenchSpeed(speed);
        } else if (speed < 0 && !_inputs.ClimbWenchOutLimitSwitch) {
            _climber.setClimbingWenchSpeed(speed);
        } else {
            _climber.stopWenchMotors();
        }

        SmartDashboard.putNumber(getName() + "/ClimberSpeed", speed);

    }

    public Command hooksManaul(double speed) {
        return Commands.run(() -> _climber.setHookMotorSpeed(speed));

    }

    //#region Commands

    public Command setClimberOutCommand() {
        return Commands.run(() -> setClimberSpeed(-ClimberMap.ClimberSpeed));
    }

    public Command setClimberInCommand() {
        return Commands.run(() -> setClimberSpeed(ClimberMap.ClimberSpeed));
    }

    public Command setHooksClosedCommand() {
        return Commands.run(() -> setHooksSpeed(ClimberMap.HooksSpeed));
    }

    public Command setHooksOpenCommand() {
        return Commands.run(() -> setHooksSpeed(-ClimberMap.HooksSpeed));
    }

    public Command stopClimbingMotorsCommand() {
        return Commands.runOnce(_climber::stopWenchMotors);
    }

    public Command stopHooksMotorsCommand() {
        return Commands.runOnce(_climber::stopHooksMotors);
    }

    public Command stopAllMotors() {
        return stopClimbingMotorsCommand().andThen(stopHooksMotorsCommand());
    }

    //#endregion
}