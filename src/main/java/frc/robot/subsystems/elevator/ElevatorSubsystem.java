
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ElevatorController;
import org.prime.control.MRSGConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
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
        public static final double MaxElevatorHeight = 0.63;
        public static final double MaxSpeedCoefficient = 0.5;
        public static final double OutputSprocketDiameterMeters = Units.Millimeters.of(32.2).in(Meters);
        public static final double GearRatio = 16;
        public static final double BottomLimitResetDebounceSeconds = 0.25;
        public static final int ElevatorEncoderCANID = 22;
        // public static final MRSGConstants ElevatorControllerConstants = new MRSGConstants(
        //         8, 4.5, 0, 1.4);

        public static final MRSGConstants ElevatorControllerConstantsSmall = new MRSGConstants(
                10.5, 4.5, 0, 1.05);

        public static final MRSGConstants ElevatorControllerConstantsMedium = new MRSGConstants(
                8, 4, 0, 0);

        // Only M and R are used.
        public static final MRSGConstants ElevatorControllerConstantsBig = new MRSGConstants(
                8.5, 3, 0, 0);

        public static final MRSGConstants ElevatorControllerConstantsAbsoultelyMassive = new MRSGConstants(7, 3, 0,
                0);

        // Manual control 
        public static final int MaxPercentOutput = 1;
        public static final double ManualSpeedLimitAbsolute = 0.5;

        // PIDF constants
        // public static final double FeedForwardKg = 0.16733;
        // public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(
        //         26,
        //         0,
        //         1.6,
        //         0,
        //         3.9,
        //         0.3,
        //         0.355);
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
            ElevatorPosition.kMid, 0.45,
            ElevatorPosition.kHigh, 0.627);

    private ElevatorInputsAutoLogged _inputs = new ElevatorInputsAutoLogged();
    private IElevator _elevatorIO;
    public ElevatorController ElevatorController = new ElevatorController(ElevatorMap.ElevatorControllerConstantsSmall,
            ElevatorMap.MaxElevatorHeight);
    private boolean _elevatorManaullyControlled = false;

    private BooleanEvent _positionResetEvent;
    private static EventLoop _eventLoop = new EventLoop();

    public ElevatorSubsystem(boolean isReal) {
        setName("Elevator");

        _elevatorIO = isReal
                ? new ElevatorReal()
                : new ElevatorSim();

        _positionResetEvent = new BooleanEvent(_eventLoop, () -> _inputs.BottomLimitSwitch)
                .debounce(ElevatorMap.BottomLimitResetDebounceSeconds)
                .rising();

        _positionResetEvent.ifHigh(() -> {
            _elevatorIO.resetEncoderPos();

        });
    }

    public boolean positionIsNear(ElevatorPosition pos) {
        return getElevatorPosition().isNear(getElevatorPositionAtLocation(pos), Units.Centimeters.of(2));
    }

    public boolean positionIsNear(double pos) {
        return getElevatorPosition().isNear(Meters.of(pos), Units.Centimeters.of(2));
    }

    public Measure<DistanceUnit> getElevatorPositionAtLocation(ElevatorPosition pos) {
        return Meters.of(_positionMap.get(pos));
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
            updateMotorVoltageWithEC();
        }
    }

    public void setMotorVoltageWithLimitSwitches(double finalOutput) {
        if (_inputs.TopLimitSwitch && finalOutput > 0) {
            finalOutput = MathUtil.clamp(finalOutput, -12, 0);
        } else if (_inputs.BottomLimitSwitch && finalOutput < 0) {
            finalOutput = MathUtil.clamp(finalOutput, 0, 12);
        }

        finalOutput = MathUtil.clamp(finalOutput, -12, 12);
        finalOutput = Math.abs(finalOutput) > 0.1 ? finalOutput : 0; // Deadband
        finalOutput = getScaledVoltage(finalOutput, _inputs.ElevatorDistanceMeters);

        SmartDashboard.putNumber(getName() + "/Output-EC", finalOutput);
        _elevatorIO.setMotorVoltages(finalOutput);
    }

    private void updateMotorVoltageWithEC() {
        var ec = ElevatorController.calculate(_inputs.ElevatorDistanceMeters, _inputs.ElevatorSpeedMetersPerSecond);

        // Within 5% of max height, reduce speed

        SmartDashboard.putNumber(getName() + "/finalOutput", ec);
        SmartDashboard.putNumber(getName() + "/ec", ec);
        setMotorVoltageWithLimitSwitches(ec);
    }

    private double getScaledVoltage(double outputVoltage, double currentPosition) {
        var lowPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight * 0.2;
        var highPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight - lowPositionScaleDownThreshold - 0.1;

        var speedScalingFactor = 1.0;
        if (currentPosition <= lowPositionScaleDownThreshold && outputVoltage < 0) {
            var scale = currentPosition / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.4, scale);
        } else if (currentPosition >= highPositionScaleDownThreshold && outputVoltage > 0) {
            var scale = (ElevatorMap.MaxElevatorHeight - currentPosition) / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.4, scale);
        }

        Logger.recordOutput(getName() + "/ scalingFactor", speedScalingFactor);
        Logger.recordOutput(getName() + "/ outputVoltage", speedScalingFactor * outputVoltage);

        return outputVoltage * speedScalingFactor;
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

        // if (_inputs.BottomLimitSwitch) {
        //     _elevatorIO.resetEncoderPos();
        // }

        _eventLoop.poll();
        Logger.recordOutput(getName() + "/elevator error", ElevatorController.getError());
        SmartDashboard.putBoolean(getName() + " is elevator manaully controlled", _elevatorManaullyControlled);
        SmartDashboard.putNumber(getName() + "EC Setpoint", ElevatorController.getSetpoint());

    }
    //#endregion

    //#region Commands

    public Command elevatorDefaultCommand(DoubleSupplier elevatorManaulControl) {
        return this.run(() -> manageElevatorControl(elevatorManaulControl.getAsDouble()));
    }

    public Command setElevatorSetpointCommand(ElevatorPosition pos) {
        return Commands.runOnce(() -> {
            System.out.println("Setting elevator position to: " + _positionMap.get(pos));
            ElevatorController.setSetpoint(_positionMap.get(pos));
        }).andThen(disableElevatorManualControlCommand()).andThen(() -> {
            if (Math.abs(_positionMap.get(pos) - _inputs.ElevatorDistanceMeters) > 0.5) {
                ElevatorController.setM(ElevatorMap.ElevatorControllerConstantsAbsoultelyMassive.M);
                ElevatorController.setR(ElevatorMap.ElevatorControllerConstantsAbsoultelyMassive.R);
            } else if (Math.abs(_positionMap.get(pos) - _inputs.ElevatorDistanceMeters) > 0.4) {
                ElevatorController.setM(ElevatorMap.ElevatorControllerConstantsBig.M);
                ElevatorController.setR(ElevatorMap.ElevatorControllerConstantsBig.R);
            } else if (Math.abs(_positionMap.get(pos) - _inputs.ElevatorDistanceMeters) > 0.2) {
                ElevatorController.setM(ElevatorMap.ElevatorControllerConstantsMedium.M);
                ElevatorController.setR(ElevatorMap.ElevatorControllerConstantsMedium.R);
            } else if (Math.abs(_positionMap.get(pos) - _inputs.ElevatorDistanceMeters) < 0.2) {
                ElevatorController.setM(ElevatorMap.ElevatorControllerConstantsSmall.M);
                ElevatorController.setR(ElevatorMap.ElevatorControllerConstantsSmall.R);
            }
        });
    }

    public Command goToElevatorBottomCommand() {
        return Commands
                .run(() -> _elevatorIO.setMotorSpeeds(-ElevatorMap.MaxSpeedCoefficient / 2))
                .until(() -> _inputs.BottomLimitSwitch)
                .withTimeout(7)
                .andThen(Commands.runOnce(() -> {
                    _elevatorIO.stopMotors();
                    ElevatorController.setSetpoint(0);
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
