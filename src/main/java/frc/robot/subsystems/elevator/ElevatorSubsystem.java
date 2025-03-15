
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ElevatorControlController;
import org.prime.control.ExtendedPIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    public static class ElevatorMap {
        public static final int LeftElevatorMotorCANID = 15;
        public static final int RightElevatorMotorCANID = 16;
        public static final int TopLimitSwitchChannel = 0;
        public static final int BottomLimitSwitchChannel = 1;
        public static final int MaxPercentOutput = 1;
        public static final double MaxElevatorHeight = 0.63;
        public static final double MaxSpeedCoefficient = 0.5;

        // public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(0.043861, 0, 0, 0,
        //         0.28922,
        //         0.024268,
        //         0.29376);

        public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(0, 0, 0.04, 0,
                0.28922 * 1.5,
                0.024268,
                0.15);

        // public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(0, 0, 0, 0,
        //         19.17 / 5,
        //         76.7 / 5,
        //         0.355);
        public static final double FeedForwardKg = 0.085;
        public static final double OutputSprocketDiameterMeters = Units.Millimeters.of(32.2).in(Meters);
        public static final double GearRatio = 16;
        public static final int ElevatorEncoderCANID = 22;
    }

    public enum ElevatorPosition {
        /** The lowest the elevator can physically go */
        kAbsoluteMinimum,
        /** The position to intake from source */
        kSource,
        /** The position to score in the trough, L1 */
        kTrough,
        /** The position to score on the level just above the trough, L2 */
        kLow,
        /** The position to score on the level two above the trought, L3 */
        kMid,
        /** The position to score on the highest level of the reef, L4 */
        kHigh
    }

    private Map<ElevatorPosition, Double> _positionMap = Map.of(
            ElevatorPosition.kAbsoluteMinimum, 0.0,
            ElevatorPosition.kSource, 0.229,
            ElevatorPosition.kTrough, 0.172,
            ElevatorPosition.kLow, 0.311,
            ElevatorPosition.kMid, 0.432,
            ElevatorPosition.kHigh, 0.627);

    private ElevatorInputsAutoLogged _inputs = new ElevatorInputsAutoLogged();
    private IElevator _elevatorIO;
    private PIDController _positionPidController;
    private ElevatorFeedforward _positionFeedforward;
    private ElevatorControlController _elevatorController = new ElevatorControlController(6.6, 4.5, 0, 1.1);
    // private ElevatorControlController _elevatorController = new ElevatorControlController(1, 1, 0.355, 1);
    private boolean _elevatorManaullyControlled = false;

    public ElevatorSubsystem(boolean isReal) {
        setName("Elevator");

        _elevatorIO = isReal
                ? new ElevatorReal()
                : new ElevatorSim();

        _positionPidController = ElevatorMap.PositionPID.createPIDController(0.02);
        _positionFeedforward = new ElevatorFeedforward(ElevatorMap.PositionPID.kS, ElevatorMap.FeedForwardKg,
                ElevatorMap.PositionPID.kV, ElevatorMap.PositionPID.kA);

        SmartDashboard.putData(_positionPidController);

    }

    public double getElevatorPositionMeters() {
        return _inputs.ElevatorDistanceMeters;
    }

    public double getElevatorPositionPercent() {
        return getElevatorPositionMeters() / ElevatorMap.MaxElevatorHeight;
    }

    //#region Control

    public void manageElevatorControl(double manualControlSpeed) {
        boolean tryingToUseManualControl = manualControlSpeed != 0 || _elevatorManaullyControlled;

        if (manualControlSpeed != 0) {
            _elevatorManaullyControlled = true;
        }

        if (tryingToUseManualControl) {
            setMotorSpeedsWithLimitSwitches(manualControlSpeed);
        } else {
            updateMotorSpeedsWithPID();
        }
    }

    public void setMotorSpeedsWithLimitSwitches(double finalOutput) {
        if (_inputs.TopLimitSwitch && finalOutput > 0) {
            finalOutput = MathUtil.clamp(finalOutput, -ElevatorMap.MaxPercentOutput, 0);
        } else if (_inputs.BottomLimitSwitch && finalOutput < 0) {
            finalOutput = MathUtil.clamp(finalOutput, 0, ElevatorMap.MaxPercentOutput);
        }

        finalOutput = MathUtil.clamp(finalOutput, -ElevatorMap.MaxSpeedCoefficient, ElevatorMap.MaxSpeedCoefficient);

        SmartDashboard.putNumber(getName() + "/Output-PIDFF", finalOutput);
        _elevatorIO.setMotorSpeeds(finalOutput);
        // _elevatorIO.setMotorVoltages(finalOutput);
    }

    private void updateMotorSpeedsWithPID() {
        // var pid = _positionPidController.calculate(_inputs.ElevatorDistanceMeters);
        // var desiredVelocity = _positionPidController.getError() * ElevatorMap.MaxSpeedCoefficient;
        // var ff = _positionFeedforward.calculate(desiredVelocity);
        // var finalOutput = MathUtil.clamp(pid + ff, -ElevatorMap.MaxSpeedCoefficient, ElevatorMap.MaxSpeedCoefficient);

        // Logger.recordOutput("Elevator/pidraw", pid + ff);
        // Logger.recordOutput("Elevator/desiredVelocity", desiredVelocity);

        var ec = _elevatorController.calculate(_inputs.ElevatorDistanceMeters, _inputs.ElevatorSpeedMetersPerSecond);
        var finalOutput = MathUtil.clamp(ec, -ElevatorMap.MaxSpeedCoefficient, ElevatorMap.MaxSpeedCoefficient);

        // Within 5% of max height, reduce speed
        finalOutput = getScaledSpeed(finalOutput, _inputs.ElevatorDistanceMeters);

        SmartDashboard.putNumber(getName() + "/finalOutput", finalOutput);
        SmartDashboard.putNumber(getName() + "/ec", ec);
        // SmartDashboard.putNumber(getName() + "/PID", pid);
        // SmartDashboard.putNumber(getName() + "/feedForward", ff);
        SmartDashboard.putNumber(getName() + "/setpoint", _positionPidController.getSetpoint());

        // setMotorSpeedsWithLimitSwitches(finalOutput);
    }

    private double getScaledSpeed(double outputSpeed, double currentPosition) {
        var lowPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight * 0.3;
        var highPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight - lowPositionScaleDownThreshold;

        var speedScalingFactor = 1.0;
        if (currentPosition <= lowPositionScaleDownThreshold && outputSpeed < 0) {
            var scale = currentPosition / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.1, scale);
        } else if (currentPosition >= highPositionScaleDownThreshold && outputSpeed > 0) {
            var scale = (ElevatorMap.MaxElevatorHeight - currentPosition) / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.1, scale);
        }

        if (speedScalingFactor < 0.1) {
            speedScalingFactor = 0.1;
        }

        return outputSpeed * speedScalingFactor;
    }

    public void setMotorVoltages(double newVolatage) {
        _elevatorIO.setMotorVoltages(newVolatage);
    }

    public void disableElevatorManaulControl() {
        _elevatorManaullyControlled = false;
    }

    @Override
    public void periodic() {
        _elevatorIO.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
        Logger.recordOutput("Elevator/ElevatorSetpoint", _positionPidController.getSetpoint());

        if (_inputs.BottomLimitSwitch) {
            _elevatorIO.resetEncoderPos();

        }

        double ec = _elevatorController.calculate(.45, _inputs.ElevatorDistanceMeters,
                _inputs.ElevatorSpeedMetersPerSecond);

        Logger.recordOutput("Elevator/EC Out", ec);

        // setMotorVoltages(ec);

    }

    public Command runWithEC() {
        return Commands.run(() -> setMotorVoltages(_elevatorController.calculate(.45, _inputs.ElevatorDistanceMeters,
                _inputs.ElevatorSpeedMetersPerSecond)));
    }

    //#endregion

    //#region Commands

    public Command ElevatorDefaultCommand(DoubleSupplier elevatorManaulControl) {
        return this.run(() -> manageElevatorControl(elevatorManaulControl.getAsDouble()));
    }

    public Command setElevatorSetpointCommand(ElevatorPosition pos) {
        return Commands.runOnce(() -> {
            System.out.println("Setting elevator position to: " + _positionMap.get(pos));
            // _positionPidController.setSetpoint(_positionMap.get(pos));
            _elevatorController.setSetpoint(_positionMap.get(pos));
        });
    }

    public Command goToElevatorBottomCommand() {
        return Commands
                .run(() -> _elevatorIO.setMotorSpeeds(-ElevatorMap.MaxSpeedCoefficient / 2))
                .until(() -> _inputs.BottomLimitSwitch)
                .withTimeout(7)
                .andThen(Commands.runOnce(() -> {
                    _elevatorIO.stopMotors();
                    _positionPidController.setSetpoint(0);
                }));
    }

    public Command disableElevatorManualControlCommand() {
        return Commands.runOnce(this::disableElevatorManaulControl);
    }

    public Command stopMotorsCommand() {
        return this.runOnce(_elevatorIO::stopMotors);
    }

    public Map<String, Command> elevatorNamedCommands() {
        return Map.of(
                "Stop Elevator Motors Command", stopMotorsCommand(),
                "Elevator High Position Command", setElevatorSetpointCommand(ElevatorPosition.kHigh),
                "Elevator Middle Position Command", setElevatorSetpointCommand(ElevatorPosition.kMid),
                "Elevator Low Position Command", setElevatorSetpointCommand(ElevatorPosition.kLow),
                "Elevator Trough Position Command", setElevatorSetpointCommand(ElevatorPosition.kTrough),
                "Elevator Source Position Command", setElevatorSetpointCommand(ElevatorPosition.kAbsoluteMinimum));
    }

    //#endregion
}
