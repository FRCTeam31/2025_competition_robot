
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

public class ElevatorSubsystem extends SubsystemBase {

    public static class ElevatorMap {
        // CAN IDs
        public static final int LeftElevatorMotorCANID = 15;
        public static final int RightElevatorMotorCANID = 16;
        public static final int ElevatorEncoderCANID = 22;

        // Limit Switch constants
        public static final int TopLimitSwitchChannel = 0;
        public static final int BottomLimitSwitchChannel = 1;
        public static final double BottomLimitResetDebounceSeconds = 0.25;

        // Physical properties
        public static final double MaxElevatorHeight = 0.63;
        public static final double OutputSprocketDiameterMeters = Units.Millimeters.of(32.2).in(Meters);
        public static final double GearRatio = 16;

        // Manual control 
        public static final int MaxPercentOutput = 1;
        public static final double ManualSpeedLimitAbsolute = 0.5;

        // PIDF constants
        public static final double FeedForwardKg = 0.16733;
        public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(
                26,
                0,
                1.6,
                0,
                3.9,
                0.3,
                0.355);
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
        kHigh;

        /**
         * Returns a string corresponding to the value of the enum. Used for filtering named
         * commands used in autonmous.
         * @return
         */
        public String getAsRawName() {
            switch (this) {
                default:
                    return "L2";
                case kSource:
                    return "Source";
                case kTrough:
                    return "Trough";
                case kLow:
                    return "L2";
                case kMid:
                    return "L3";
                case kHigh:
                    return "L4";
            }
        }

        public static ElevatorPosition getFromRawName(String name) {
            switch (name) {
                case "Source":
                    return kSource;
                case "Trough":
                    return kTrough;
                case "L2":
                    return kLow;
                case "L3":
                    return kMid;
                case "L4":
                    return kHigh;
            }

            return null;
        }
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
    private TrapezoidProfile.Constraints _trapezoidConstraints;
    private ProfiledPIDController _pidController;
    private ElevatorFeedforward _feedforward;
    private boolean _elevatorManaullyControlled = false;
    private BooleanEvent _positionResetEvent;

    public ElevatorSubsystem(boolean isReal) {
        setName("Elevator");

        _elevatorIO = isReal
                ? new ElevatorReal()
                : new ElevatorSim();

        _trapezoidConstraints = new TrapezoidProfile.Constraints(ElevatorMap.MaxElevatorHeight,
                ElevatorMap.MaxElevatorHeight * 4);
        _pidController = new ProfiledPIDController(
                ElevatorMap.PositionPID.kP,
                ElevatorMap.PositionPID.kI,
                ElevatorMap.PositionPID.kD,
                _trapezoidConstraints,
                0.02);
        _feedforward = new ElevatorFeedforward(
                ElevatorMap.PositionPID.kS,
                ElevatorMap.FeedForwardKg,
                ElevatorMap.PositionPID.kV,
                ElevatorMap.PositionPID.kA);

        _positionResetEvent = new BooleanEvent(Robot.EventLoop, () -> _inputs.BottomLimitSwitch)
                .debounce(ElevatorMap.BottomLimitResetDebounceSeconds)
                .rising();
        _positionResetEvent.ifHigh(() -> {
            _elevatorIO.resetEncoderPos();
        });
    }

    public Measure<DistanceUnit> getElevatorPositionAtLocation(ElevatorPosition pos) {
        return Meters.of(_positionMap.get(pos));
    }

    public boolean positionIsNear(ElevatorPosition pos) {
        return getElevatorPosition().isNear(getElevatorPositionAtLocation(pos), Units.Centimeters.of(2));
    }

    public double getElevatorPositionMeters() {
        return _inputs.ElevatorDistanceMeters;
    }

    public Distance getElevatorPosition() {
        return Meters.of(getElevatorPositionMeters());
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
            setMotorVoltageWithLimitSwitches(manualControlSpeed * 12);
        } else {
            var pid = _pidController.calculate(_inputs.ElevatorDistanceMeters);
            var ff = _feedforward.calculateWithVelocities(_inputs.ElevatorSpeedMetersPerSecond,
                    _pidController.getSetpoint().velocity);
            var output = pid + ff;

            SmartDashboard.putNumber(getName() + "/finalOutput", output);
            setMotorVoltageWithLimitSwitches(output);
        }
    }

    public void setMotorVoltageWithLimitSwitches(double finalOutput) {
        if (_inputs.TopLimitSwitch && finalOutput > 0) {
            finalOutput = MathUtil.clamp(finalOutput, -12, 0);
        } else if (_inputs.BottomLimitSwitch && finalOutput < 0) {
            finalOutput = MathUtil.clamp(finalOutput, 0, 12);
        }

        finalOutput = MathUtil.clamp(finalOutput, -12, 12);
        finalOutput = getScaledOutput(finalOutput, _inputs.ElevatorDistanceMeters);
        finalOutput = MathUtil.applyDeadband(finalOutput, 0.1);

        SmartDashboard.putNumber(getName() + "/Output-EC", finalOutput);
        _elevatorIO.setMotorVoltages(finalOutput);
    }

    private double getScaledOutput(double output, double currentPosition) {
        var lowPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight * 0.3;
        var highPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight - lowPositionScaleDownThreshold;

        var speedScalingFactor = 1.0;
        if (currentPosition <= lowPositionScaleDownThreshold && output < 0) {
            speedScalingFactor = currentPosition / lowPositionScaleDownThreshold;
        } else if (currentPosition >= highPositionScaleDownThreshold && output > 0) {
            speedScalingFactor = (ElevatorMap.MaxElevatorHeight - currentPosition) / lowPositionScaleDownThreshold;
        }

        speedScalingFactor = Math.max(0.1, speedScalingFactor);
        return output * speedScalingFactor;
    }

    public void setMotorVoltages(double newVolatage) {
        _elevatorIO.setMotorVoltages(newVolatage);
    }

    public void disableElevatorManaulControl() {
        _elevatorManaullyControlled = false;
    }

    public boolean atPositionSetpoint() {
        return _pidController.atGoal();
    }

    @Override
    public void periodic() {
        _elevatorIO.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);

        SmartDashboard.putData(getName() + "/elevator_pid_controller", _pidController);
        Logger.recordOutput(getName() + "/elevator-pid-pos-error", _pidController.getPositionError());
        Logger.recordOutput(getName() + "/elevator-pid-vel-error", _pidController.getVelocityError());
        Logger.recordOutput(getName() + "/elevator-pid-setpoint", _pidController.getSetpoint().position);
        SmartDashboard.putBoolean(getName() + " is elevator manaully controlled", _elevatorManaullyControlled);

    }
    //#endregion

    //#region Commands

    public Command manageControlCommand(DoubleSupplier elevatorManaulControl) {
        return this.run(() -> manageElevatorControl(elevatorManaulControl.getAsDouble()));
    }

    public Command setElevatorSetpointCommand(ElevatorPosition pos) {
        return Commands.print("Setting elevator position to: " + _positionMap.get(pos))
                .andThen(Commands.runOnce(() -> _pidController.setGoal(_positionMap.get(pos))))
                .andThen(disableElevatorManualControlCommand())
                .andThen(Commands.waitUntil(this::atPositionSetpoint));
    }

    public Command goToElevatorBottomCommand() {
        return Commands
                .run(() -> _elevatorIO.setMotorSpeeds(-ElevatorMap.ManualSpeedLimitAbsolute / 2))
                .until(() -> _inputs.BottomLimitSwitch)
                .withTimeout(7)
                .andThen(Commands.runOnce(() -> {
                    _elevatorIO.stopMotors();
                    _pidController.setGoal(0);
                }));
    }

    public Command disableElevatorManualControlCommand() {
        return Commands.runOnce(this::disableElevatorManaulControl);
    }

    public Command stopMotorsCommand() {
        return this.runOnce(_elevatorIO::stopMotors);
    }

    public Map<String, Command> getNamedCommands() {
        return Map.of(
                "Elevator/Stop Motors", stopMotorsCommand(),
                "Elevator/Goto High", setElevatorSetpointCommand(ElevatorPosition.kHigh),
                "Elevator/Goto Middle", setElevatorSetpointCommand(ElevatorPosition.kMid),
                "Elevator/Goto Low", setElevatorSetpointCommand(ElevatorPosition.kLow),
                "Elevator/Goto Trough", setElevatorSetpointCommand(ElevatorPosition.kTrough),
                "Elevator/Goto Source", setElevatorSetpointCommand(ElevatorPosition.kAbsoluteMinimum));
    }

    //#endregion
}
