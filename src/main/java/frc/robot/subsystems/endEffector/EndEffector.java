package frc.robot.subsystems.endEffector;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Container;
import frc.robot.subsystems.elevator.ElevatorPosition;

public class EndEffector extends SubsystemBase {
    private IEndEffector _endEffector;
    private PIDController _wristPID;
    private boolean _wristManuallyControlled = false;

    // private LockableEvent<ElevatorPosition> _lockableSetpoint = new LockableEvent<>(
    //         null,
    //         this::setWristSetpointCommand,
    //         true);

    private Map<ElevatorPosition, Double> _angleAtElevatorHeight = Map.of(
            ElevatorPosition.kAbsoluteMinimum, 0.0,
            ElevatorPosition.kSource, -42.36,
            ElevatorPosition.kTrough, -92.0,
            ElevatorPosition.kL2, -126.0,
            ElevatorPosition.kL3, -126.0,
            ElevatorPosition.kL4, -121.0);

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    private boolean _intakeIsEjecting = false;
    private double _manualControlSpeed = 0;
    private double _wristSetpoint = 0;

    // TODO: Add a system to set the wrist angle to a predefined setpoint. Look at the system used in the elevator subsystem for reference. We will most likely use the same enums used in the elevator subsystem so it is easier to implement the auto rotating system later. For now, create a temporary enum.

    public EndEffector(boolean isReal) {
        setName("End Effector");
        _endEffector = isReal
                ? new EndEffectorReal()
                : new EndEffectorSim();

        _wristPID = EndEffectorMap.WristPID.createPIDController(0.02);
        SmartDashboard.putData(_wristPID);
    }

    public void manageWristControl() {
        var inDangerZone = Container.Elevator.getElevatorPositionMeters() <= EndEffectorMap.LowerElevatorSafetyLimit;
        var isSafeForManualControl = !inDangerZone
                && _inputs.EndEffectorAngleDegrees <= EndEffectorMap.WristMaxManuallyControllableAngle;

        var tryingToUseManualControl = (_manualControlSpeed != 0 || _wristManuallyControlled);
        _wristManuallyControlled = (isSafeForManualControl && tryingToUseManualControl);

        if (_wristManuallyControlled && !inDangerZone) {
            runWristManual(_manualControlSpeed);
        } else {
            seekWristAnglePID(inDangerZone);
        }
    }

    public void seekWristAnglePID(boolean inDangerZone) {
        SmartDashboard.putBoolean(getName() + "/InDangerZone", inDangerZone);

        var safeAngle = calculateDangerZoneAngle(Container.Elevator.getElevatorPositionMeters());
        SmartDashboard.putNumber(getName() + "/dangerZoneCalculatedSafeAngle", safeAngle);

        SmartDashboard.putNumber(getName() + "/PID-Setpoint", _wristSetpoint);
        double pid = inDangerZone
                ? _wristPID.calculate(_inputs.EndEffectorAngleDegrees, 0) // Revert instructions: set this to 0
                : _wristPID.calculate(_inputs.EndEffectorAngleDegrees,
                        Math.max(EndEffectorMap.WristMaxAngle, _wristSetpoint));

        // if (!inDangerZone) {
        //     double previousSetpoint = _wristPID.getSetpoint();
        //     _wristPID.setSetpoint(
        //             Math.min(previousSetpoint, EndEffectorMap.WristMaxAngle));
        // }

        SmartDashboard.putNumber(getName() + "/PID-Raw", pid);
        pid = MathUtil.clamp(pid, -EndEffectorMap.WristMaxOutput, EndEffectorMap.WristMaxOutput);

        SmartDashboard.putNumber(getName() + "/PID-FinalOutput", pid);
        _endEffector.setWristSpeed(pid);
    }

    private double calculateDangerZoneAngle(double currentAngle) {
        var t1 = 6686.42684 * Math.pow(currentAngle, 2);
        var t2 = 1875.25759 * currentAngle;
        var t3 = 5.58733;

        return t1 - t2 - t3;
    }

    public void runWristManual(double speed) {
        double manualMotorControl = -speed * EndEffectorMap.WristMaxOutput;

        // Artifically Limit the wrist angle
        if (_inputs.EndEffectorAngleDegrees <= EndEffectorMap.WristMinAngle) {
            manualMotorControl = Math.max(manualMotorControl, 0);
        } else if (_inputs.EndEffectorAngleDegrees >= EndEffectorMap.WristMaxAngle) {
            manualMotorControl = Math.min(manualMotorControl, 0);
        }
        _endEffector.setWristSpeed(manualMotorControl);
    }

    private void disableWristManualControl() {
        _wristManuallyControlled = false;
    }

    private void manageIntakeSpeeds(boolean runIntakeOut) {
        // Driver is trying to eject && InTeleop
        if (runIntakeOut && DriverStation.isTeleopEnabled()) {
            _endEffector.setIntakeSpeed(EndEffectorMap.EjectSpeed);
            return;
        }

        // Use instance variable (used in automation)
        if (_intakeIsEjecting) {
            _endEffector.setIntakeSpeed(EndEffectorMap.EjectSpeed);
            return;
        }

        if (_inputs.CoralLimitSwitchState) {
            _endEffector.stopIntakeMotor();
        }
    }

    @Override
    public void periodic() {
        _endEffector.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
        SmartDashboard.putBoolean(getName() + "wristManuallyControlled", _wristManuallyControlled);

        manageWristControl();
    }

    public boolean wristAtSetpoint() {
        return Math.abs(_inputs.EndEffectorAngleDegrees - _wristSetpoint) < 3;
    }

    /**
     * Default command which runs the wrist PID and runs the intake based on two buttons
     * @param runIntakeIn
     * @param runIntakeOut
     */
    public Command defaultCommand(BooleanSupplier runIntakeOut, DoubleSupplier wristManualControl) {
        return this.run(() -> {
            manageIntakeSpeeds(runIntakeOut.getAsBoolean());
            _manualControlSpeed = MathUtil.applyDeadband(wristManualControl.getAsDouble(), 0.05);
        });
    }

    /**
    
     * Sets the wrist setpoint
     * @param angle Desired angle
     */
    public Command setWristSetpointCommand(double angle) {
        return Commands.runOnce(this::disableWristManualControl)
                // .andThen(() -> _wristPID.setSetpoint(angle));
                .andThen(() -> _wristSetpoint = angle);
    }

    /**
    
     * Sets the wrist setpoint
     * @param position Elevator position
     */
    public Command setWristSetpointCommand(ElevatorPosition position) {
        return Commands.runOnce(this::disableWristManualControl)
                .andThen(setWristSetpointCommand(_angleAtElevatorHeight.get(position)));
    }

    // /**
    //  * Schedules the setting of the wrist setpoint. If in a danger zone, the setpoint will be locked until the danger zone is exited.
    //  * Only then it will set the setpoint. If not in a danger zone, it will set the setpoint immediately.
    //  * 
    //  * @param position
    //  * @return
    //  */
    // public Command scheduleWristSetpointCommand(ElevatorPosition position) {
    //     // return Commands.runOnce(this::disableWristManualControlCommand).andThen(() -> {
    //     //     // _lockableSetpoint.setEvent(position);
    //     //     // System.out.println("Event: " + position);
    //     // });
    //     //         // .andThen(_lockableSetpoint.scheduleLockableEventCommand())
    //     //         // .andThen(() -> System.out.println(
    //     //         //         _lockableSetpoint.getEvent().toString() + "\n" + _lockableSetpoint.isLocked().toString()));

    //     // // TODO: May have to use .finallyDo() instead of .andThen()

    //     return Commands.runOnce(this::disableWristManualControlCommand)
    //             .andThen(Commands.waitUntil(() -> !_isLocked)
    //                     .andThen(setWristSetpointCommand(position)));
    // }

    public Command wristManualControlCommand(double speed) {
        return Commands.runOnce(() -> _endEffector.setWristSpeed(speed));
    }

    public Command disableWristManualControlCommand() {
        return Commands.runOnce(this::disableWristManualControl);
    }

    public Command stopIntakeMotorCommand() {
        return Commands.runOnce(() -> _endEffector.stopIntakeMotor());
    }

    public Command stopWristMotorCommand() {
        return Commands.runOnce(() -> _endEffector.stopWristMotor());
    }

    public Command stopBothMotorsCommand() {
        return this.runOnce(() -> _endEffector.stopMotors());
    }

    public Command enableIntakeCommand() {
        return this.runOnce(() -> {
            _endEffector.setIntakeSpeed(EndEffectorMap.IntakeSpeed);
            _intakeIsEjecting = false;
        });
    }

    public Command enableEjectCommand() {
        return this.runOnce(() -> {
            _endEffector.setIntakeSpeed(EndEffectorMap.EjectSpeed);
            _intakeIsEjecting = true;
        });
    }

    public Command disableEjectCommand() {
        return this.runOnce(() -> {
            _endEffector.stopIntakeMotor();
            _intakeIsEjecting = false;
        });
    }

    public Command scoreCoral() {
        // return enableIntakeCommand() // intake until wrist is at setpoint
        return Commands.waitUntil(this::wristAtSetpoint)
                .andThen(enableEjectCommand()) // eject until limit switch is let go (or times out)
                .until(() -> !_inputs.CoralLimitSwitchState).withTimeout(2)
                .andThen(Commands.waitSeconds(0.75)) // slight delay to allow the coral to begin ejecting
                .andThen(setWristSetpointCommand(-30)) // set the angle up while ejecting
                .andThen(Commands.waitSeconds(0.75))
                .andThen(disableEjectCommand());
        // .andThen(enableIntakeCommand());
    }

    public Command pickupCoral() {
        return enableIntakeCommand()
                .andThen(Commands.waitUntil(() -> _inputs.CoralLimitSwitchState))
                .andThen(disableEjectCommand())
                .andThen(setWristSetpointCommand(0))
                .andThen(Commands.print(">> INTAKE: Coral picked up"));
    }
}