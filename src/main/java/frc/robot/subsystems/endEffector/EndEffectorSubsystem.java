package frc.robot.subsystems.endEffector;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.util.LockableEvent;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Container;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;

public class EndEffectorSubsystem extends SubsystemBase {

    public class EndEffectorMap {
        public static final byte IntakeMotorCanID = 19;
        public static final byte WristMotorCanID = 20;
        public static final byte LimitSwitchDIOChannel = 9;
        public static final double EjectSpeed = 0.5;
        public static final double IntakeSpeed = -1;
        public static final byte IntakeCurrentLimit = 20;
        public static final byte WristCurrentLimit = 30;
        public static final ExtendedPIDConstants WristPID = new ExtendedPIDConstants(0.007, 0, 0);
        public static final double GearRatio = 20;
        public static final double MinWristAngle = -135;
        public static final double MaxWristAngle = 0;

        // Wrist Constants
        public static final double WristUp = 0;
        public static final double WristSource = 45;
        public static final double WristReef = 135;
        public static final double WristMaxOutput = 0.175;

        public static final double LowerElevatorHeightLimit = 0.14;
        public static final double UpperElevatorHeightLimit = 0.4;
        public static final double SafeWristAngleAtUpperElevatorHeightLimit = -30;
    }

    private IEndEffector _endEffector;
    private PIDController _wristPID;

    private boolean _wristManuallyControlled = false;

    private LockableEvent<ElevatorPosition> _lockableSetpoint = new LockableEvent<>(null, this::setWristSetpointCommand,
            true);

    private Map<ElevatorPosition, Double> _angleAtElevatorHeight = Map.of(
            ElevatorPosition.kAbsoluteMinimum, 0.0,
            ElevatorPosition.kSource, -42.36,
            ElevatorPosition.kTrough, -92.0,
            ElevatorPosition.kLow, -126.0,
            ElevatorPosition.kMid, -126.0,
            ElevatorPosition.kHigh, -130.0);

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    // TODO: Add a system to set the wrist angle to a predefined setpoint. Look at the system used in the elevator subsystem for reference. We will most likely use the same enums used in the elevator subsystem so it is easier to implement the auto rotating system later. For now, create a temporary enum.

    public EndEffectorSubsystem(boolean isReal) {
        setName("End Effector");
        _endEffector = isReal ? new EndEffectorReal() : new EndEffectorSim();
        _wristPID = EndEffectorMap.WristPID.createPIDController(0.02);
        SmartDashboard.putData(_wristPID);
    }

    public void manageWristControl(double manaulControlSpeed) {

        boolean tryingToUseManualControl = manaulControlSpeed != 0 || _wristManuallyControlled;
        //Store if the wrist is currently manaully controlled, this will not be disabled until operator clicks the controller stick button
        _wristManuallyControlled = manaulControlSpeed != 0;

        boolean aboveMinHeightThreshold = Container.Elevator
                .getElevatorPositionMeters() >= EndEffectorMap.LowerElevatorHeightLimit;

        if (tryingToUseManualControl && aboveMinHeightThreshold) {
            runWristManaull(manaulControlSpeed);
        } else {
            seekWristAngle();
        }

    }

    public void seekWristAngle() {
        var belowMinHeightThreshold = Container.Elevator
                .getElevatorPositionMeters() <= EndEffectorMap.LowerElevatorHeightLimit;
        var aboveMaxHeightThreshold = Container.Elevator
                .getElevatorPositionMeters() >= EndEffectorMap.UpperElevatorHeightLimit;

        boolean inDangerZone = belowMinHeightThreshold || aboveMaxHeightThreshold;

        double pid = belowMinHeightThreshold
                ? _wristPID.calculate(_inputs.EndEffectorAngleDegrees, 0)
                : _wristPID.calculate(_inputs.EndEffectorAngleDegrees);

        if (aboveMaxHeightThreshold && !belowMinHeightThreshold) {
            double previousSetpoint = _wristPID.getSetpoint();
            _wristPID.setSetpoint(
                    Math.min(previousSetpoint, EndEffectorMap.SafeWristAngleAtUpperElevatorHeightLimit));
        }

        SmartDashboard.putNumber(getName() + "/WristPIDRaw", pid);

        pid = MathUtil.clamp(pid, -EndEffectorMap.WristMaxOutput, EndEffectorMap.WristMaxOutput);

        SmartDashboard.putNumber(getName() + "/WristPID-FinalOutput", pid);
        SmartDashboard.putNumber(getName() + "/WristSetpoint", _wristPID.getSetpoint());

        _endEffector.setWristSpeed(pid);

        if (inDangerZone) {
            _lockableSetpoint.lock();
        } else {
            _lockableSetpoint.unlock();
        }
    }

    public void runWristManaull(double speed) {
        double manualMotorControl = -speed * EndEffectorMap.WristMaxOutput;
        boolean aboveMaxHeightThreshold = Container.Elevator
                .getElevatorPositionMeters() >= EndEffectorMap.UpperElevatorHeightLimit;

        // Artifically Limit the wrist angle
        if (_inputs.EndEffectorAngleDegrees <= EndEffectorMap.MinWristAngle) {
            manualMotorControl = Math.max(manualMotorControl, 0);
        } else if (_inputs.EndEffectorAngleDegrees >= EndEffectorMap.MaxWristAngle) {
            manualMotorControl = Math.min(manualMotorControl, 0);
        } else if (aboveMaxHeightThreshold
                && _inputs.EndEffectorAngleDegrees >= EndEffectorMap.SafeWristAngleAtUpperElevatorHeightLimit) {
            manualMotorControl = Math.min(manualMotorControl, 0);
        }

        _endEffector.setWristSpeed(manualMotorControl);
    }

    private void disableWristManaulControl() {
        _wristManuallyControlled = false;
    }

    @Override
    public void periodic() {
        _endEffector.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);

        Logger.recordOutput("End Effector/Wrist Setpoint", _wristPID.getSetpoint());
        Logger.recordOutput("End Effector/Scheduled Wrist Setpoint", _lockableSetpoint.getEvent());
    }

    /**
     * Default command which runs the wrist PID and runs the intake based on two buttons
     * @param runIntakeIn
     * @param runIntakeOut
     */
    public Command defaultCommand(BooleanSupplier runIntakeIn, BooleanSupplier runIntakeOut,
            DoubleSupplier wristManualControl) {
        return this.run(() -> {
            if (runIntakeIn.getAsBoolean()) {
                _endEffector.setIntakeSpeed(EndEffectorMap.IntakeSpeed);

            } else if (runIntakeOut.getAsBoolean()) {
                _endEffector.setIntakeSpeed(EndEffectorMap.EjectSpeed);

            } else {
                _endEffector.stopIntakeMotor();
            }

            seekWristAngle(wristManualControl.getAsDouble());
        });
    }

    /**
    
     * Sets the wrist setpoint
     * @param angle Desired angle
     */
    public Command setWristSetpointCommand(double angle) {
        return Commands.runOnce(this::disableWristManaulControl).andThen(() -> {
            _wristPID.setSetpoint(angle);
        });
    }

    /**
    
     * Sets the wrist setpoint
     * @param position Elevator position
     */
    public Command setWristSetpointCommand(ElevatorPosition position) {
        return Commands.runOnce(this::disableWristManaulControl).andThen(() -> {
            _wristPID.setSetpoint(_angleAtElevatorHeight.get(position));
        });
    }

    /**
     * Schedules the setting of the wrist setpoint. If in a danger zone, the setpoint will be locked until the danger zone is exited.
     * Only then it will set the setpoint. If not in a danger zone, it will set the setpoint immediately.
     * 
     * @param position
     * @return
     */
    public Command scheduleWristSetpointCommand(ElevatorPosition position) {
        return Commands.runOnce(this::disableWristManaulControl).andThen(() -> {
            _lockableSetpoint.setEvent(position);
            _lockableSetpoint.schedule();
        });
    }

    public Command wristManualControlCommand(double speed) {
        return Commands.runOnce(() -> _endEffector.setWristSpeed(speed));
    }

    public Command resetWristManualControlCommand() {
        return Commands.runOnce(this::disableWristManaulControl);
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

    public Command setIntakeSpeedCommand(double speed) {
        return this.run(() -> _endEffector.setIntakeSpeed(speed));
    }

    // TODO: Add more named commands for running the intake and rotating the wrist to predefined setpoints (See above todo)
    public Map<String, Command> getNamedCommands() {
        return Map.of("Stop Intake", stopIntakeMotorCommand(), "Stop Both Motors", stopBothMotorsCommand(),
                "Intake Coral",
                setIntakeSpeedCommand(EndEffectorMap.IntakeSpeed), "Eject Coral",
                setIntakeSpeedCommand(EndEffectorMap.EjectSpeed));
    }

}