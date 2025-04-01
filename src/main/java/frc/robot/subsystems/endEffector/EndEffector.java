package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.util.MotorUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Container;
import frc.robot.subsystems.elevator.ElevatorPosition;

public class EndEffector extends SubsystemBase {
    private IEndEffector _endEffector;
    private PIDController _wristPID;

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    private boolean _wristManuallyControlled = false;
    private boolean _intakeIsEjecting = false;
    private double _manualControlSpeed = 0;

    private LEDPattern _waitingForCoralLEDPattern = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.3));

    public EndEffector(boolean isReal) {
        setName("End Effector");
        _endEffector = isReal
                ? new EndEffectorReal()
                : new EndEffectorSim();

        _wristPID = EndEffectorMap.WristPID.createPIDController(0.02);
        SmartDashboard.putData(getName() + "/Wrist/PID", _wristPID);
    }

    //#region Control Methods

    /** 
     * Returns true if the wrist is at its PID setpoint 
     */
    public boolean wristAtSetpoint() {
        var delta = _inputs.EndEffectorAngleDegrees - _wristPID.getSetpoint();
        return Math.abs(delta) < EndEffectorMap.WristTolerance;
    }

    /** 
     * Returns true if the coral limit switch is triggered
     */
    public boolean coralLimitSwitchTriggered() {
        return _inputs.CoralLimitSwitchState;
    }

    /**
     * Returns true if the wrist is safe for manual control
     */
    private boolean isSafeForManualControl() {
        var belowSafetyLimit = Container.Elevator
                .getElevatorPositionMeters() <= EndEffectorMap.LowerElevatorSafetyLimit;
        var wristAngleSafe = _inputs.EndEffectorAngleDegrees <= EndEffectorMap.WristMaxManuallyControllableAngle;

        return belowSafetyLimit && wristAngleSafe;
    }

    /**
     * Returns true if the elevator is in the danger zone
     */
    private boolean inElevatorDangerZone() {
        return Container.Elevator.getElevatorPositionMeters() <= EndEffectorMap.LowerElevatorSafetyLimit;
    }

    /**
     * Manages the wrist control, either using manual control or PID control
     */
    private void controlWristMovement() {
        _wristManuallyControlled = (isSafeForManualControl() && _manualControlSpeed != 0);

        if (_wristManuallyControlled) {
            var manualSpeed = -_manualControlSpeed * EndEffectorMap.WristMaxOutput;
            boolean revLimitReached = _inputs.EndEffectorAngleDegrees <= EndEffectorMap.WristMinAngle;
            boolean fwdLimitReached = _inputs.EndEffectorAngleDegrees >= EndEffectorMap.WristMaxAngle;

            _endEffector.setWristSpeed(
                    MotorUtil.getMotorspeedWithLimits(manualSpeed, fwdLimitReached, revLimitReached));
        } else {
            var safeAngle = calculateDangerZoneAngle(Container.Elevator.getElevatorPositionMeters()); // TODO: Unused right now, only logged - needs testing
            double pid = inElevatorDangerZone()
                    ? _wristPID.calculate(_inputs.EndEffectorAngleDegrees, 0)
                    : _wristPID.calculate(_inputs.EndEffectorAngleDegrees);

            pid = MathUtil.applyDeadband(pid, 0.1);
            pid = MathUtil.clamp(pid, -EndEffectorMap.WristMaxOutput, EndEffectorMap.WristMaxOutput);
            _endEffector.setWristSpeed(pid);

            Logger.recordOutput(getName() + "/Wrist/pid-output-raw", pid);
            Logger.recordOutput(getName() + "/Wrist/scaled-safe-angle", safeAngle);
        }
    }

    /**
     * Calculates the safe angle for the wrist to be at when the elevator is in the danger zone
     * @param elevatorHeight The height of the elevator in meters
     * @return
     */
    private double calculateDangerZoneAngle(double elevatorHeight) {
        return EndEffectorMap.ElevatorDangerZoneWristAngleLookup.get(elevatorHeight);
    }

    /**
     * Sets the wrist setpoint using an angle and disables manual control
     * @param angle
     */
    private void setWristSetpoint(double angle) {
        _wristPID.setSetpoint(angle);
        _wristManuallyControlled = false;
    }

    //#endregion

    @Override
    public void periodic() {
        _endEffector.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
        Logger.recordOutput(getName() + "/Wrist/setpoint", _wristPID.getSetpoint());
        Logger.recordOutput(getName() + "/Wrist/manually-controlled", _wristManuallyControlled);

        controlWristMovement();
    }

    /**
     * Default command which runs the wrist PID and runs the intake based on two buttons
     * @param runIntakeIn
     * @param ejectSupplier
     */
    public Command defaultCommand(BooleanSupplier ejectSupplier, DoubleSupplier wristManualControl) {
        return this.run(() -> {
            // Check if driver is manually ejecting in Teleop
            if (ejectSupplier.getAsBoolean() && DriverStation.isTeleopEnabled()) {
                _endEffector.setIntakeSpeed(EndEffectorMap.EjectSpeed);
            } else {
                // If not ejecting, run the intake either at intake speed or eject speed based on the state
                _endEffector.setIntakeSpeed(_intakeIsEjecting
                        ? EndEffectorMap.EjectSpeed
                        : EndEffectorMap.IntakeSpeed);
            }

            _manualControlSpeed = wristManualControl.getAsDouble();
        });
    }

    /**
     * Sets the wrist setpoint using an angle
     * @param angle Desired angle
     */
    public Command setWristSetpointCommand(double angle) {
        return Commands.runOnce(this::disableEjectCommand)
                .andThen(() -> setWristSetpoint(angle));
    }

    /**
     * Sets the wrist setpoint using an ElevatorPosition
     * @param position Elevator position
     */
    public Command setWristSetpointCommand(ElevatorPosition position) {
        return setWristSetpointCommand(EndEffectorMap.ElevatorHeightWristAngleMap.get(position));
    }

    /**
     * Stops the intake motor
     */
    public Command stopIntakeMotorCommand() {
        return Commands.runOnce(() -> _endEffector.stopIntakeMotor());
    }

    /**
     * Stops the wrist motor
     */
    public Command stopWristMotorCommand() {
        return Commands.runOnce(() -> _endEffector.stopWristMotor());
    }

    /**
     * Stops both motors
     */
    public Command stopBothMotorsCommand() {
        return this.runOnce(() -> _endEffector.stopMotors());
    }

    /**
     * Disables intake ejection
     */
    public Command disableEjectCommand() {
        return this.runOnce(() -> _intakeIsEjecting = false);
    }

    /**
     * Enables intake ejection
     */
    public Command enableEjectCommand() {
        return this.runOnce(() -> _intakeIsEjecting = true);
    }

    /**
     * Scores coral by ejecting it until the limit switch is let go, then pulling the wrist back to -30 degrees
     * @return
     */
    public Command scoreCoral() {
        return disableEjectCommand() // Maintain intake until wrist is at setpoint
                .andThen(Commands.waitUntil(this::wristAtSetpoint))
                // Eject until limit switch is let go (or times out)
                .andThen(enableEjectCommand())
                .until(() -> !_inputs.CoralLimitSwitchState).withTimeout(2)
                // Slight delay to allow the coral to begin ejecting
                .andThen(Commands.waitSeconds(0.75))
                // Set the angle up while ejecting to "flick" the coral out
                .andThen(setWristSetpointCommand(EndEffectorMap.WristMaxManuallyControllableAngle))
                // Wait for the wrist to reach the setpoint
                .andThen(Commands.waitUntil(this::wristAtSetpoint))
                // Reset back to intaking
                .andThen(disableEjectCommand());
    }

    public Command pickupCoral() {
        return disableEjectCommand()
                .andThen(Commands.waitUntil(this::coralLimitSwitchTriggered))
                .andThen(setWristSetpointCommand(0))
                .andThen(Commands.print(">> INTAKE: Coral picked up"));
    }
}