
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ElevatorController;
import org.prime.control.SubsystemControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SuperStructure;

public class Elevator extends SubsystemBase {
    public IElevator _elevatorIO;
    public ElevatorController _elevatorController = new ElevatorController(ElevatorMap.MRSGConstantsSmall);
    private BooleanEvent _positionResetEvent;
    private double _manualControlSpeed = 0;

    public Elevator(boolean isReal) {
        setName("Elevator");

        _elevatorIO = isReal
                ? new ElevatorReal()
                : new ElevatorSim();

        _positionResetEvent = new BooleanEvent(Robot.EventLoop,
                () -> SuperStructure.ElevatorState.BottomLimitSwitch)
                .debounce(ElevatorMap.BottomLimitResetDebounceSeconds)
                .rising();

        _positionResetEvent.ifHigh(_elevatorIO::resetEncoderPos);
    }

    //#region Properties

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
        return Meters.of(ElevatorMap.PositionMap.get(pos));
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
        return getElevatorPositionMeters() / ElevatorMap.MaxHeight;
    }

    //#endregion

    //#region Control

    /**
     * Sets the motor voltage, respecting the limit switches
     * @param voltage
     */
    private void setMotorVoltageWithLimitSwitches(double voltage) {
        Logger.recordOutput(getName() + "/output-voltage-unfiltered", voltage);

        // If the elevator is at the top or bottom, stop the motor from moving in that direction
        if (SuperStructure.ElevatorState.TopLimitSwitch && voltage > 0) {
            voltage = MathUtil.clamp(voltage, -12, 0);
        } else if (SuperStructure.ElevatorState.BottomLimitSwitch && voltage < 0) {
            voltage = MathUtil.clamp(voltage, 0, 12);
        }

        voltage = MathUtil.clamp(voltage, -12, 12);
        // Within 5% of max height, reduce speed
        voltage = scaleOutputApproachingLimits(voltage, SuperStructure.ElevatorState.DistanceMeters);
        voltage = MathUtil.applyDeadband(voltage, 0.1); // Deadband at 10%

        Logger.recordOutput(getName() + "/output-voltage-filtered", voltage);
        _elevatorIO.setMotorVoltages(voltage);
    }

    private double scaleOutputApproachingLimits(double output, double currentPosition) {
        var lowPositionScaleDownThreshold = ElevatorMap.MaxHeight * 0.2;
        var highPositionScaleDownThreshold = ElevatorMap.MaxHeight - lowPositionScaleDownThreshold - 0.1;

        var speedScalingFactor = 1.0;
        if (currentPosition <= lowPositionScaleDownThreshold && output < 0) {
            var scale = currentPosition / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.4, scale);
        } else if (currentPosition >= highPositionScaleDownThreshold && output > 0) {
            var scale = (ElevatorMap.MaxHeight - currentPosition) / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.4, scale);
        }

        return output * speedScalingFactor;
    }

    public void setPositionSetpoint(ElevatorPosition pos) {
        _elevatorController.setSetpoint(ElevatorMap.PositionMap.get(pos));
        SuperStructure.ElevatorState.ControlMode = SubsystemControlMode.PIDControlled;

        if (Math.abs(ElevatorMap.PositionMap.get(pos) - SuperStructure.ElevatorState.DistanceMeters) > 0.5) {
            _elevatorController.setM(ElevatorMap.MRSGConstantsAbsoultelyMassive.M);
            _elevatorController.setR(ElevatorMap.MRSGConstantsAbsoultelyMassive.R);
        } else if (Math.abs(ElevatorMap.PositionMap.get(pos) - SuperStructure.ElevatorState.DistanceMeters) > 0.4) {
            _elevatorController.setM(ElevatorMap.MRSGConstantsBig.M);
            _elevatorController.setR(ElevatorMap.MRSGConstantsBig.R);
        } else if (Math.abs(ElevatorMap.PositionMap.get(pos) - SuperStructure.ElevatorState.DistanceMeters) > 0.2) {
            _elevatorController.setM(ElevatorMap.MRSGConstantsMedium.M);
            _elevatorController.setR(ElevatorMap.MRSGConstantsMedium.R);
        } else if (Math.abs(ElevatorMap.PositionMap.get(pos) - SuperStructure.ElevatorState.DistanceMeters) < 0.2) {
            _elevatorController.setM(ElevatorMap.MRSGConstantsSmall.M);
            _elevatorController.setR(ElevatorMap.MRSGConstantsSmall.R);
        }
    }

    @Override
    public void periodic() {
        _elevatorIO.updateInputs(SuperStructure.ElevatorState);
        Logger.processInputs(getName(), SuperStructure.ElevatorState);

        // Manually logging the elevator controller state
        Logger.recordOutput(getName() + "/closed-loop-setpoint", _elevatorController.getSetpoint());
        Logger.recordOutput(getName() + "/closed-loop-error", _elevatorController.getError());

        // Control the elevator based on the current control mode
        switch (SuperStructure.ElevatorState.ControlMode) {
            case ManuallyControlled:
                var manualControlVolts = _manualControlSpeed * 12;
                setMotorVoltageWithLimitSwitches(_manualControlSpeed < 0
                        ? (manualControlVolts * 0.50) // Slow down speed when going down
                        : manualControlVolts);
                break;
            default:
                var closedLoopOutput = _elevatorController.calculate(
                        SuperStructure.ElevatorState.DistanceMeters,
                        SuperStructure.ElevatorState.SpeedMPS);

                setMotorVoltageWithLimitSwitches(closedLoopOutput);
                break;
        }
    }
    //#endregion

    //#region Commands

    public Command elevatorDefaultCommand(DoubleSupplier elevatorManualControl) {
        return this.run(() -> {
            _manualControlSpeed = elevatorManualControl.getAsDouble();

            if (Math.abs(_manualControlSpeed) > 0.05) {
                SuperStructure.ElevatorState.ControlMode = SubsystemControlMode.ManuallyControlled;
            }
        });
    }

    public Command setElevatorSetpointCommand(ElevatorPosition pos) {
        return Commands.print("Setting elevator position to: " + ElevatorMap.PositionMap.get(pos))
                .andThen(() -> setPositionSetpoint(pos));
    }

    public Command homeLiftCommand() {
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
        return Commands.runOnce(() -> SuperStructure.ElevatorState.ControlMode = SubsystemControlMode.PIDControlled);
    }

    public Command stopMotorsCommand() {
        return this.runOnce(() -> {
            _elevatorIO.stopMotors();
            _manualControlSpeed = 0;
            SuperStructure.ElevatorState.ControlMode = SubsystemControlMode.ManuallyControlled;
        });
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
