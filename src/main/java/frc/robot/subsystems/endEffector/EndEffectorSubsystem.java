package frc.robot.subsystems.endEffector;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.util.LockableEvent;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Container;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorPosition;

public class EndEffectorSubsystem extends SubsystemBase {

    public class EndEffectorMap {
        public static final byte IntakeMotorCanID = 19;
        public static final byte WristMotorCanID = 20;
        public static final byte LimitSwitchDIOChannel = 9;
        public static final double EjectSpeed = -1;
        public static final double IntakeSpeed = 1;
        public static final byte IntakeCurrentLimit = 20;
        public static final byte WristCurrentLimit = 30;
        public static final ExtendedPIDConstants WristPID = new ExtendedPIDConstants(0.01, 0, 0);
        public static final double GearRatio = 20;
        public static final double MinWristAngle = -135;
        public static final double MaxWristAngle = -30;
        public static final double WristNonVerticalAngleThreshold = -110;

        // Wrist Constants
        public static final double WristUp = 0;
        public static final double WristSource = 45;
        public static final double WristReef = 135;
        public static final double WristMaxOutput = 0.175;

        public static final double LowerElevatorSafetyLimit = 0.15;
    }

    private IEndEffector _endEffector;
    private PIDController _wristPID;
    private boolean _wristManuallyControlled = false;

    private BooleanEvent _elevatorReachedTroughHeight;
    private BooleanEvent _elevatorReachedLowHeight;
    private BooleanEvent _elevatorReachedMidHeight;
    private BooleanEvent _elevatorReachedHighHeight;
    private BooleanEvent _elevatorReachedSourceHeight;
    private BooleanEvent _elevatorBelowDangerZone;

    private LockableEvent<ElevatorPosition> _lockableSetpoint = new LockableEvent<>(
            null,
            this::setWristSetpointCommand,
            true);

    private Map<ElevatorPosition, Double> _angleAtElevatorHeight = Map.of(
            ElevatorPosition.kAbsoluteMinimum, 0.0,
            ElevatorPosition.kSource, -42.36,
            ElevatorPosition.kTrough, -92.0,
            ElevatorPosition.kL2, -126.0,
            ElevatorPosition.kL3, -126.0,
            ElevatorPosition.kL4, -121.0);

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    private boolean _isLocked = true;
    private boolean _intakeIsEjecting = false;
    private double _manualControlSpeed = 0;

    // TODO: Add a system to set the wrist angle to a predefined setpoint. Look at the system used in the elevator subsystem for reference. We will most likely use the same enums used in the elevator subsystem so it is easier to implement the auto rotating system later. For now, create a temporary enum.

    public EndEffectorSubsystem(boolean isReal) {
        setName("End Effector");
        _endEffector = isReal
                ? new EndEffectorReal()
                : new EndEffectorSim();

        // TODO: Use the below events to trigger automatic movement of the end effector, when using PID control
        _elevatorReachedLowHeight = new BooleanEvent(Robot.EventLoop,
                () -> Container.Elevator.positionIsNear(ElevatorPosition.kL2))
                .rising()
                .debounce(0.5);
        _elevatorReachedMidHeight = new BooleanEvent(Robot.EventLoop,
                () -> Container.Elevator.positionIsNear(ElevatorPosition.kL3))
                .rising()
                .debounce(0.5);
        _elevatorReachedHighHeight = new BooleanEvent(Robot.EventLoop,
                () -> Container.Elevator.positionIsNear(ElevatorPosition.kL4))
                .rising()
                .debounce(0.5);
        _elevatorReachedSourceHeight = new BooleanEvent(Robot.EventLoop,
                () -> Container.Elevator.positionIsNear(ElevatorPosition.kSource))
                .rising()
                .debounce(0.5);
        _elevatorReachedTroughHeight = new BooleanEvent(Robot.EventLoop,
                () -> Container.Elevator.positionIsNear(ElevatorPosition.kTrough))
                .rising()
                .debounce(0.5);
        _elevatorBelowDangerZone = new BooleanEvent(Robot.EventLoop, () -> Container.Elevator
                .getElevatorPositionMeters() <= EndEffectorMap.LowerElevatorSafetyLimit)
                .rising()
                .debounce(1);

        _wristPID = EndEffectorMap.WristPID.createPIDController(0.02);
        SmartDashboard.putData(_wristPID);
    }

    public void manageWristControl() {
        boolean isSafeForManualControl = Container.Elevator
                .getElevatorPositionMeters() >= EndEffectorMap.LowerElevatorSafetyLimit
                && _inputs.EndEffectorAngleDegrees <= EndEffectorMap.MaxWristAngle + 5;

        boolean tryingToUseManualControl = (_manualControlSpeed != 0 || _wristManuallyControlled);

        _wristManuallyControlled = (isSafeForManualControl && tryingToUseManualControl) ? true : false;

        if (_wristManuallyControlled) {
            runWristManual(_manualControlSpeed);
        } else if (!_wristManuallyControlled) {
            seekWristAnglePID();
        }
    }

    public void seekWristAnglePID() {
        var inDangerZone = Container.Elevator
                .getElevatorPositionMeters() <= EndEffectorMap.LowerElevatorSafetyLimit;

        var safeAngle = calculateDangerZoneAngle(Container.Elevator
                .getElevatorPositionMeters());
        SmartDashboard.putNumber(getName() + "/dangerZoneCalculatedSafeAngle", safeAngle);

        double pid = inDangerZone
                ? _wristPID.calculate(_inputs.EndEffectorAngleDegrees, 0) // Revert instructions: set this to 0
                : _wristPID.calculate(_inputs.EndEffectorAngleDegrees);

        if (!inDangerZone) {
            double previousSetpoint = _wristPID.getSetpoint();
            _wristPID.setSetpoint(
                    Math.min(previousSetpoint, EndEffectorMap.MaxWristAngle));
        }

        SmartDashboard.putNumber(getName() + "/WristPIDRaw", pid);

        pid = MathUtil.clamp(pid, -EndEffectorMap.WristMaxOutput, EndEffectorMap.WristMaxOutput);

        SmartDashboard.putNumber(getName() + "/WristPID-FinalOutput", pid);
        SmartDashboard.putNumber(getName() + "/WristSetpoint", _wristPID.getSetpoint());

        _endEffector.setWristSpeed(pid);

        if (inDangerZone) {
            // _lockableSetpoint.lock();
            _isLocked = true;
        } else {
            // _lockableSetpoint.unlock();
            _isLocked = false;
        }
    }

    private double calculateDangerZoneAngle(double currentAngle) {$
        var t1 = 6686.42684 * Math.pow(currentAngle, 2);
        var t2 = 1875.25759 * currentAngle;
        var t3 = 5.58733;

        return t1 - t2 - t3;
    }

    public void runWristManual(double speed) {
        double manualMotorControl = -speed * EndEffectorMap.WristMaxOutput;

        // Artifically Limit the wrist angle
        if (_inputs.EndEffectorAngleDegrees <= EndEffectorMap.MinWristAngle) {
            manualMotorControl = Math.max(manualMotorControl, 0);
        } else if (_inputs.EndEffectorAngleDegrees >= EndEffectorMap.MaxWristAngle) {
            manualMotorControl = Math.min(manualMotorControl, 0);
        }
        _endEffector.setWristSpeed(manualMotorControl);
    }

    private void disableWristManualControl() {
        _wristManuallyControlled = false;
    }

    @Override
    public void periodic() {
        _endEffector.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);

        Logger.recordOutput("End Effector/Wrist Setpoint", _wristPID.getSetpoint());
        // Logger.recordOutput("End Effector/Scheduled Wrist Setpoint", _lockableSetpoint.getEvent());
        SmartDashboard.putBoolean(getName() + "wristManuallyControlled", _wristManuallyControlled);

        SmartDashboard.putNumber(getName() + " Wrist setpoint", _wristPID.getSetpoint());
        SmartDashboard.putBoolean("EndEffector/isLocked", _lockableSetpoint.isLocked().getAsBoolean());
        manageWristControl();
    }

    public boolean wristAtSetpoint() {
        return Math.abs(_inputs.EndEffectorAngleDegrees - _wristPID.getSetpoint()) < 3;
    }

    /**
     * Default command which runs the wrist PID and runs the intake based on two buttons
     * @param runIntakeIn
     * @param runIntakeOut
     */
    public Command defaultCommand(BooleanSupplier runIntakeOut,
            DoubleSupplier wristManualControl) {
        return this.run(() -> {
            if (runIntakeOut.getAsBoolean() && DriverStation.isTeleopEnabled()) {
                _endEffector.setIntakeSpeed(EndEffectorMap.EjectSpeed);
            } else {
                _endEffector.setIntakeSpeed(_intakeIsEjecting
                        ? EndEffectorMap.EjectSpeed
                        : EndEffectorMap.IntakeSpeed);
            }

            _manualControlSpeed = MathUtil.applyDeadband(wristManualControl.getAsDouble(), 0.05);
        });
    }

    /**
    
     * Sets the wrist setpoint
     * @param angle Desired angle
     */
    public Command setWristSetpointCommand(double angle) {
        return Commands.runOnce(this::disableWristManualControl)
                .andThen(enableIntakeCommand())
                .andThen(() -> _wristPID.setSetpoint(angle));
    }

    /**
    
     * Sets the wrist setpoint
     * @param position Elevator position
     */
    public Command setWristSetpointCommand(ElevatorPosition position) {
        return Commands.runOnce(this::disableWristManualControl)
                .andThen(setWristSetpointCommand(_angleAtElevatorHeight.get(position)));
    }

    /**
     * Schedules the setting of the wrist setpoint. If in a danger zone, the setpoint will be locked until the danger zone is exited.
     * Only then it will set the setpoint. If not in a danger zone, it will set the setpoint immediately.
     * 
     * @param position
     * @return
     */
    public Command scheduleWristSetpointCommand(ElevatorPosition position) {
        // return Commands.runOnce(this::disableWristManualControlCommand).andThen(() -> {
        //     // _lockableSetpoint.setEvent(position);
        //     // System.out.println("Event: " + position);
        // });
        //         // .andThen(_lockableSetpoint.scheduleLockableEventCommand())
        //         // .andThen(() -> System.out.println(
        //         //         _lockableSetpoint.getEvent().toString() + "\n" + _lockableSetpoint.isLocked().toString()));

        // // TODO: May have to use .finallyDo() instead of .andThen()

        return Commands.runOnce(this::disableWristManualControlCommand)
                .andThen(Commands.waitUntil(() -> !_isLocked)
                        .andThen(setWristSetpointCommand(position)));
    }

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

    public Command scoreCoral() {
        return enableIntakeCommand() // intake until wrist is at setpoint
                .andThen(Commands.waitUntil(this::wristAtSetpoint))
                .andThen(enableEjectCommand()) // eject until limit switch is let go (or times out)
                .until(() -> !_inputs.CoralLimitSwitchState).withTimeout(2)
                .andThen(Commands.waitSeconds(0.75)) // slight delay to allow the coral to begin ejecting
                .andThen(setWristSetpointCommand(-30)) // set the angle up while ejecting
                .andThen(Commands.waitSeconds(0.75))
                .andThen(enableIntakeCommand());
    }

    public Command pickupCoral() {
        return enableIntakeCommand()
                .andThen(Commands.waitUntil(() -> _inputs.CoralLimitSwitchState))
                .andThen(setWristSetpointCommand(0))
                .andThen(Commands.print(">> INTAKE: Coral picked up"));
    }

    // TODO: Add more named commands for running the intake and rotating the wrist to predefined setpoints (See above todo)
    public Map<String, Command> getNamedCommands() {
        return Map.of("Stop Intake", stopIntakeMotorCommand(),
                "Stop Both Motors", stopBothMotorsCommand(),
                "Intake Coral", enableIntakeCommand(),
                "Eject Coral", enableEjectCommand());
    }

}