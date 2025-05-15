
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ElevatorController;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.SuperStructure;

public class Elevator extends SubsystemBase {
    public Map<ElevatorPosition, Double> _positionMap = Map.of(
            ElevatorPosition.kAbsoluteMinimum, 0.0,
            ElevatorPosition.kSource, 0.16,
            ElevatorPosition.kTrough, 0.172,
            ElevatorPosition.kL2, 0.28,
            ElevatorPosition.kL3, 0.432,
            ElevatorPosition.kL4, 0.627);

    public IElevator _elevatorIO;
    public ElevatorController _elevatorController = new ElevatorController(
            ElevatorMap.ElevatorControllerConstantsSmall);
    private boolean _elevatorManaullyControlled = false;

    private BooleanEvent _positionResetEvent;

    public Elevator(boolean isReal) {
        setName("Elevator");

        _elevatorIO = isReal
                ? new ElevatorReal()
                : new ElevatorSim();

        _positionResetEvent = new BooleanEvent(Robot.EventLoop, () -> SuperStructure.ElevatorState.BottomLimitSwitch)
                .debounce(ElevatorMap.BottomLimitResetDebounceSeconds)
                .rising();

        _positionResetEvent.ifHigh(_elevatorIO::resetEncoderPos);
    }

    /**
     * Returns true if the elevator is at its PID setpoint
     * @return
     */
    public boolean atSetpoint() {
        return _elevatorController.atSetpoint(0.02);
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
        return SuperStructure.ElevatorState.DistanceMeters;
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
    public void controlElevator(double manualControlSpeed) {
        _elevatorManaullyControlled = manualControlSpeed != 0 || _elevatorManaullyControlled;

        if (_elevatorManaullyControlled) {
            var manualControlVolts = manualControlSpeed * 12;
            setMotorVoltageWithLimitSwitches(manualControlSpeed < 0
                    ? (manualControlVolts * 0.50)
                    : manualControlVolts);
        } else {
            var ec = _elevatorController.calculate(
                    SuperStructure.ElevatorState.DistanceMeters,
                    SuperStructure.ElevatorState.SpeedMPS);

            SmartDashboard.putNumber(getName() + "/Raw-EC", ec);
            setMotorVoltageWithLimitSwitches(ec);
        }
    }

    /**
     * Sets the motor voltage, respecting the limit switches
     * @param finalOutput
     */
    private void setMotorVoltageWithLimitSwitches(double finalOutput) {
        Logger.recordOutput(getName() + "/output-raw", finalOutput);
        // If the elevator is at the top or bottom, stop the motor from moving in that direction
        if (SuperStructure.ElevatorState.TopLimitSwitch && finalOutput > 0) {
            finalOutput = MathUtil.clamp(finalOutput, -12, 0);
        } else if (SuperStructure.ElevatorState.BottomLimitSwitch && finalOutput < 0) {
            finalOutput = MathUtil.clamp(finalOutput, 0, 12);
        }

        finalOutput = MathUtil.clamp(finalOutput, -12, 12);
        // Within 5% of max height, reduce speed
        finalOutput = scaleOutputApproachingLimits(finalOutput, SuperStructure.ElevatorState.DistanceMeters);
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

    public void setPositionSetpoint(ElevatorPosition pos) {
        _elevatorController.setSetpoint(_positionMap.get(pos));
        disableElevatorManualControl();

        if (Math.abs(_positionMap.get(pos) - SuperStructure.ElevatorState.DistanceMeters) > 0.5) {
            _elevatorController.setM(ElevatorMap.ElevatorControllerConstantsAbsoultelyMassive.M);
            _elevatorController.setR(ElevatorMap.ElevatorControllerConstantsAbsoultelyMassive.R);
        } else if (Math.abs(_positionMap.get(pos) - SuperStructure.ElevatorState.DistanceMeters) > 0.4) {
            _elevatorController.setM(ElevatorMap.ElevatorControllerConstantsBig.M);
            _elevatorController.setR(ElevatorMap.ElevatorControllerConstantsBig.R);
        } else if (Math.abs(_positionMap.get(pos) - SuperStructure.ElevatorState.DistanceMeters) > 0.2) {
            _elevatorController.setM(ElevatorMap.ElevatorControllerConstantsMedium.M);
            _elevatorController.setR(ElevatorMap.ElevatorControllerConstantsMedium.R);
        } else if (Math.abs(_positionMap.get(pos) - SuperStructure.ElevatorState.DistanceMeters) < 0.2) {
            _elevatorController.setM(ElevatorMap.ElevatorControllerConstantsSmall.M);
            _elevatorController.setR(ElevatorMap.ElevatorControllerConstantsSmall.R);
        }
    }

    @Override
    public void periodic() {
        _elevatorIO.updateInputs(SuperStructure.ElevatorState);
        Logger.processInputs(getName(), SuperStructure.ElevatorState);
        Logger.recordOutput(getName() + "EC-setpoint", _elevatorController.getSetpoint());
        Logger.recordOutput(getName() + "/EC-error", _elevatorController.getError());

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
                .until(() -> SuperStructure.ElevatorState.BottomLimitSwitch)
                .withTimeout(7)
                .andThen(Commands.runOnce(() -> {
                    _elevatorIO.stopMotors();
                    _elevatorController.setSetpoint(0);
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
