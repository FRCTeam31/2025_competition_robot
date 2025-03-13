package frc.robot.subsystems.endEffector;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Container;

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

        public static final double MaxWristAngle = -135;
        public static final double MinWristAngle = 0;

        // Wrist Constants
        public static final double WristUp = 0;
        public static final double WristSource = 45;
        public static final double WristReef = 135;
        public static final double WristMaxOutput = 0.175;

        public static final double LowerElevatorHeightLimit = 0.11;
        public static final double UpperElevatorHeightLimit = 0.38;
        public static final double SafeWristAngleAtUpperElevatorHeightLimit = -30;
    }

    private IEndEffector _endEffector;
    private PIDController _wristPID;

    private boolean _endEffectorManuallyControlled = false;

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    // TODO: Add a system to set the wrist angle to a predefined setpoint. Look at the system used in the elevator subsystem for reference. We will most likely use the same enums used in the elevator subsystem so it is easier to implement the auto rotating system later. For now, create a temporary enum.

    public EndEffectorSubsystem(boolean isReal) {
        setName("End Effector");
        _endEffector = isReal ? new EndEffectorReal() : new EndEffectorSim();
        _wristPID = EndEffectorMap.WristPID.createPIDController(0.02);
        SmartDashboard.putData(_wristPID);
    }

    public void seekWristAngle(double endEffectorManualControl) {
        var belowMinHeightThreshold = Container.Elevator
                .getElevatorPositionMeters() <= EndEffectorMap.LowerElevatorHeightLimit;
        var aboveMaxHeightThreshold = Container.Elevator
                .getElevatorPositionMeters() >= EndEffectorMap.UpperElevatorHeightLimit;

        var inDangerZone = belowMinHeightThreshold || aboveMaxHeightThreshold ? true : false;

        double pid = belowMinHeightThreshold
                ? _wristPID.calculate(_inputs.EndEffectorAngleDegrees, 0)
                : _wristPID.calculate(_inputs.EndEffectorAngleDegrees);

        if (aboveMaxHeightThreshold && !belowMinHeightThreshold) {
            double previousSetpoint = _wristPID.getSetpoint();
            _wristPID.setSetpoint(
                    Math.min(previousSetpoint, EndEffectorMap.SafeWristAngleAtUpperElevatorHeightLimit));
        }

        SmartDashboard.putNumber(getName() + "/WristPID", pid);
        SmartDashboard.putNumber(getName() + "/WristSetpoint", _wristPID.getSetpoint());

        pid = MathUtil.clamp(pid, -EndEffectorMap.WristMaxOutput, EndEffectorMap.WristMaxOutput);

        Logger.recordOutput(getName() + "/WristPID-FinalOutput", pid);

        if ((endEffectorManualControl != 0
                || _endEffectorManuallyControlled) && !inDangerZone) {

            double manualMotorControl = -endEffectorManualControl * EndEffectorMap.WristMaxOutput;

            if (_inputs.EndEffectorAngleDegrees <= EndEffectorMap.MaxWristAngle) {
                manualMotorControl = Math.max(manualMotorControl, 0);
            } else if (_inputs.EndEffectorAngleDegrees >= EndEffectorMap.MinWristAngle) {
                manualMotorControl = Math.min(manualMotorControl, 0);
            }

            _endEffector.setWristSpeed(manualMotorControl);
            _endEffectorManuallyControlled = true;
        } else {
            _endEffector.setWristSpeed(pid);
        }
    }

    public void resetWristManualControl() {
        _endEffectorManuallyControlled = false;
    }

    @Override
    public void periodic() {
        _endEffector.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
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
        _endEffectorManuallyControlled = false;
        return Commands.runOnce(() -> _wristPID.setSetpoint(angle));
    }

    public Command wristManualControlCommand(double speed) {
        return Commands.runOnce(() -> _endEffector.setWristSpeed(speed));
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