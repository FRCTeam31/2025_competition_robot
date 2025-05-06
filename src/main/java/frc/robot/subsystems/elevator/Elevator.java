
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
    private Map<ElevatorPosition, Double> _positionMap = Map.of(
            ElevatorPosition.kAbsoluteMinimum, 0.0,
            ElevatorPosition.kSource, 0.16,
            ElevatorPosition.kTrough, 0.172,
            ElevatorPosition.kL2, 0.28,
            ElevatorPosition.kL3, 0.432,
            ElevatorPosition.kL4, 0.627);

    private ElevatorInputsAutoLogged _inputs = new ElevatorInputsAutoLogged();
    private IElevator _elevatorIO;

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(1, 1, 1);

    // Create a motion profile with the given maximum voltage and characteristics kV, kA
    // These gains should match your feedforward kV, kA for best results.
    private final ExponentialProfile m_profile = new ExponentialProfile(
            ExponentialProfile.Constraints.fromCharacteristics(10, 1, 1));
    private ExponentialProfile.State m_goal = new ExponentialProfile.State(0, 0);
    private ExponentialProfile.State m_setpoint = new ExponentialProfile.State(0, 0);

    private final PIDController _elevatorPidController = ElevatorMap.PositionPID.createPIDController(0.02);

    private boolean _elevatorManaullyControlled = false;

    private BooleanEvent _positionResetEvent;

    public Elevator(boolean isReal) {
        setName("Elevator");

        _elevatorIO = isReal
                ? new ElevatorReal()
                : new ElevatorSim();

        _positionResetEvent = new BooleanEvent(Robot.EventLoop, () -> _inputs.BottomLimitSwitch)
                .debounce(ElevatorMap.BottomLimitResetDebounceSeconds)
                .rising();

        _positionResetEvent.ifHigh(_elevatorIO::resetEncoderPos);
    }

    /**
     * Returns true if the elevator is at its PID setpoint
     * @return
     */
    public boolean atSetpoint() {
        return _elevatorPidController.atSetpoint();
    }

    /**
     * Returns true when the elevator is withing a given distance from a position
     * @param pos
     * @param withinCentimeters
     * @return
     */
    public boolean positionIsNear(ElevatorPosition pos, double withinCentimeters) {
        return getElevatorPosition()
                .isNear(getElevatorPositionAtLocation(pos), Units.Centimeters.of(withinCentimeters));
    }

    /**
     * Returns true when the elevator is within 2cm of a given position
     * @param pos
     * @return
     */
    public boolean positionIsNear(double pos) {
        return getElevatorPosition().isNear(Meters.of(pos), Units.Centimeters.of(2));
    }

    /**
     * Gets the position of the elevator at a given location
     * @param pos
     * @return
     */
    public Measure<DistanceUnit> getElevatorPositionAtLocation(ElevatorPosition pos) {
        return Meters.of(_positionMap.get(pos));
    }

    /**
     * Gets the current position of the elevator in meters
     * @return
     */
    public double getElevatorPositionMeters() {
        return _inputs.ElevatorDistanceMeters;
    }

    /**
     * Gets the current position of the elevator as a Distance object in Meters
     * @return
     */
    public Distance getElevatorPosition() {
        return Meters.of(getElevatorPositionMeters());
    }

    /**
     * Gets the current position of the elevator as a percentage of the max height
     * @return
     */
    public double getElevatorPositionPercent() {
        return getElevatorPositionMeters() / ElevatorMap.MaxElevatorHeight;
    }

    //#region Control

    /**
     * 
     * @param manualControlSpeed
     */
    private void controlElevator(double manualControlSpeed) {
        _elevatorManaullyControlled = manualControlSpeed != 0 || _elevatorManaullyControlled;

        if (_elevatorManaullyControlled) {
            var manualControlVolts = manualControlSpeed * 12;
            setMotorVoltageWithLimitSwitches(manualControlSpeed < 0
                    ? (manualControlVolts * 0.50)
                    : manualControlVolts);
        } else {
            // Retrieve the profiled setpoint for the next timestep. This setpoint moves
            // toward the goal while obeying the constraints.
            ExponentialProfile.State next = m_profile.calculate(0.02, m_setpoint, m_goal);

            // Send setpoint to offboard controller PID
            _elevatorPidController.setSetpoint(
                    m_setpoint.position);

            var ff = m_feedforward.calculate(next.velocity) / 12.0;
            var pid = _elevatorPidController.calculate(_inputs.ElevatorDistanceMeters);

            m_setpoint = next;

            SmartDashboard.putNumber(getName() + "/ffPID", ff + pid);
            setMotorVoltageWithLimitSwitches(ff + pid);
        }
    }

    /**
     * Sets the motor voltage, respecting the limit switches
     * @param finalOutput
     */
    private void setMotorVoltageWithLimitSwitches(double finalOutput) {
        Logger.recordOutput(getName() + "/output-raw", finalOutput);
        // If the elevator is at the top or bottom, stop the motor from moving in that direction
        if (_inputs.TopLimitSwitch && finalOutput > 0) {
            finalOutput = MathUtil.clamp(finalOutput, -12, 0);
        } else if (_inputs.BottomLimitSwitch && finalOutput < 0) {
            finalOutput = MathUtil.clamp(finalOutput, 0, 12);
        }

        finalOutput = MathUtil.clamp(finalOutput, -12, 12);
        // Within 5% of max height, reduce speed
        finalOutput = scaleOutputApproachingLimits(finalOutput, _inputs.ElevatorDistanceMeters);
        finalOutput = MathUtil.applyDeadband(finalOutput, 0.1); // Deadband at 10%

        Logger.recordOutput(getName() + "/output-final", finalOutput);
        _elevatorIO.setMotorVoltages(finalOutput);
    }

    private double scaleOutputApproachingLimits(double output, double currentPosition) {
        var lowPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight * 0.2;
        var highPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight - lowPositionScaleDownThreshold - 0.1;

        var speedScalingFactor = 1.0;
        if (currentPosition <= lowPositionScaleDownThreshold && output < 0) {
            var scale = currentPosition / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.4, scale);
        } else if (currentPosition >= highPositionScaleDownThreshold && output > 0) {
            var scale = (ElevatorMap.MaxElevatorHeight - currentPosition) / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.4, scale);
        }

        Logger.recordOutput(getName() + "/output-limit-scalar", speedScalingFactor);
        Logger.recordOutput(getName() + "/output-limit-scaled", speedScalingFactor * output);

        return output * speedScalingFactor;
    }

    private void disableElevatorManualControl() {
        _elevatorManaullyControlled = false;
    }

    private void setPositionSetpoint(ElevatorPosition pos) {
        m_goal = new ExponentialProfile.State(_positionMap.get(pos), 0);
        disableElevatorManualControl();

    }

    @Override
    public void periodic() {
        _elevatorIO.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
        Logger.recordOutput(getName() + "/PID-setpoint", _elevatorPidController.getSetpoint());
        Logger.recordOutput(getName() + "/PID-error", _elevatorPidController.getError());

        SmartDashboard.putBoolean(getName() + " is elevator manaully controlled", _elevatorManaullyControlled);
    }
    //#endregion

    //#region Commands

    public Command elevatorDefaultCommand(DoubleSupplier elevatorManualControl) {
        return this.run(() -> controlElevator(elevatorManualControl.getAsDouble()));
    }

    public Command setElevatorSetpointCommand(ElevatorPosition pos) {
        return Commands.print("Setting elevator position to: " + _positionMap.get(pos))
                .andThen(() -> setPositionSetpoint(pos));
    }

    public Command goToElevatorBottomCommand() {
        return Commands
                .run(() -> _elevatorIO.setMotorSpeeds(-ElevatorMap.MaxSpeedCoefficient / 2))
                .until(() -> _inputs.BottomLimitSwitch)
                .withTimeout(7)
                .andThen(Commands.runOnce(() -> {
                    _elevatorIO.stopMotors();
                    _elevatorPidController.setSetpoint(0);
                }));
    }

    public Command disableElevatorManualControlCommand() {
        return Commands.runOnce(this::disableElevatorManualControl);
    }

    public Command stopMotorsCommand() {
        return this.runOnce(_elevatorIO::stopMotors);
    }

    public Map<String, Command> getNamedCommands() {
        return Map.of(
                "Elevator/Stop", stopMotorsCommand(),
                "Elevator/Goto High", setElevatorSetpointCommand(ElevatorPosition.kL4),
                "Elevator/Goto Middle", setElevatorSetpointCommand(ElevatorPosition.kL3),
                "Elevator/Goto Low", setElevatorSetpointCommand(ElevatorPosition.kL2),
                "Elevator/Goto Trough", setElevatorSetpointCommand(ElevatorPosition.kTrough),
                "Elevator/Goto Source", setElevatorSetpointCommand(ElevatorPosition.kAbsoluteMinimum));
    }

    //#endregion
}
