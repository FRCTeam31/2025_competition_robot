
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ElevatorController;
import org.prime.control.MRSGConstants;

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

public class ElevatorSubsystem extends SubsystemBase {

    public static class ElevatorMap {
        // CAN ids
        public static final int LeftElevatorMotorCANID = 15;
        public static final int RightElevatorMotorCANID = 16;
        public static final int ElevatorEncoderCANID = 22;

        // Limit switch constants
        public static final int TopLimitSwitchChannel = 0;
        public static final int BottomLimitSwitchChannel = 1;
        public static final double BottomLimitResetDebounceSeconds = 0.25;

        // Physical constraints
        public static final double MaxElevatorHeight = 0.63;
        public static final double MaxSpeedCoefficient = 0.5;
        public static final double OutputSprocketDiameterMeters = Units.Millimeters.of(32.2).in(Meters);
        public static final double GearRatio = 16;

        // MRSG constants
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

    private Map<ElevatorPosition, Double> _positionMap = Map.of(
            ElevatorPosition.kAbsoluteMinimum, 0.0,
            ElevatorPosition.kSource, 0.198,
            ElevatorPosition.kTrough, 0.172,
            ElevatorPosition.kL2, 0.311,
            ElevatorPosition.kL3, 0.428,
            ElevatorPosition.kL4, 0.627);

    private ElevatorInputsAutoLogged _inputs = new ElevatorInputsAutoLogged();
    private IElevator _elevatorIO;
    public ElevatorController _elevatorController = new ElevatorController(ElevatorMap.ElevatorControllerConstantsSmall,
            ElevatorMap.MaxElevatorHeight);
    private boolean _elevatorManaullyControlled = false;

    private BooleanEvent _positionResetEvent;

    public ElevatorSubsystem(boolean isReal) {
        setName("Elevator");

        _elevatorIO = isReal
                ? new ElevatorReal()
                : new ElevatorSim();

        _positionResetEvent = new BooleanEvent(Robot.EventLoop, () -> _inputs.BottomLimitSwitch)
                .debounce(ElevatorMap.BottomLimitResetDebounceSeconds)
                .rising();

        _positionResetEvent.ifHigh(_elevatorIO::resetEncoderPos);
    }

    public boolean atSetpoint() {
        return _elevatorController.atSetpoint(0.02);
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
        _elevatorManaullyControlled = manualControlSpeed != 0 || _elevatorManaullyControlled;

        if (_elevatorManaullyControlled) {
            setMotorVoltageWithLimitSwitches((manualControlSpeed * 12) * 0.75);
        } else {
            var ec = _elevatorController.calculate(_inputs.ElevatorDistanceMeters,
                    _inputs.ElevatorSpeedMetersPerSecond);

            SmartDashboard.putNumber(getName() + "/Raw-EC", ec);
            setMotorVoltageWithLimitSwitches(ec);
        }
    }

    public void setMotorVoltageWithLimitSwitches(double finalOutput) {
        if (_inputs.TopLimitSwitch && finalOutput > 0) {
            finalOutput = MathUtil.clamp(finalOutput, -12, 0);
        } else if (_inputs.BottomLimitSwitch && finalOutput < 0) {
            finalOutput = MathUtil.clamp(finalOutput, 0, 12);
        }

        finalOutput = MathUtil.clamp(finalOutput, -12, 12);
        // Within 5% of max height, reduce speed
        finalOutput = getScaledoutput(finalOutput, _inputs.ElevatorDistanceMeters);
        finalOutput = MathUtil.applyDeadband(finalOutput, 0.1); // Deadband

        SmartDashboard.putNumber(getName() + "/Filtered-EC", finalOutput);
        _elevatorIO.setMotorVoltages(finalOutput);
    }

    private double getScaledoutput(double output, double currentPosition) {
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

        Logger.recordOutput(getName() + "/output-scalingFactor", speedScalingFactor);
        Logger.recordOutput(getName() + "/output-scaled", speedScalingFactor * output);

        return output * speedScalingFactor;
    }

    public void setMotorVoltages(double newVolatage) {
        _elevatorIO.setMotorVoltages(newVolatage);
    }

    public void disableElevatorManualControl() {
        _elevatorManaullyControlled = false;
    }

    public void setPositionSetpoint(ElevatorPosition pos) {
        _elevatorController.setSetpoint(_positionMap.get(pos));
        disableElevatorManualControl();

        if (Math.abs(_positionMap.get(pos) - _inputs.ElevatorDistanceMeters) > 0.5) {
            _elevatorController.setM(ElevatorMap.ElevatorControllerConstantsAbsoultelyMassive.M);
            _elevatorController.setR(ElevatorMap.ElevatorControllerConstantsAbsoultelyMassive.R);
        } else if (Math.abs(_positionMap.get(pos) - _inputs.ElevatorDistanceMeters) > 0.4) {
            _elevatorController.setM(ElevatorMap.ElevatorControllerConstantsBig.M);
            _elevatorController.setR(ElevatorMap.ElevatorControllerConstantsBig.R);
        } else if (Math.abs(_positionMap.get(pos) - _inputs.ElevatorDistanceMeters) > 0.2) {
            _elevatorController.setM(ElevatorMap.ElevatorControllerConstantsMedium.M);
            _elevatorController.setR(ElevatorMap.ElevatorControllerConstantsMedium.R);
        } else if (Math.abs(_positionMap.get(pos) - _inputs.ElevatorDistanceMeters) < 0.2) {
            _elevatorController.setM(ElevatorMap.ElevatorControllerConstantsSmall.M);
            _elevatorController.setR(ElevatorMap.ElevatorControllerConstantsSmall.R);
        }
    }

    @Override
    public void periodic() {
        _elevatorIO.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);

        Logger.recordOutput(getName() + "/elevator error", _elevatorController.getError());
        SmartDashboard.putBoolean(getName() + " is elevator manaully controlled", _elevatorManaullyControlled);
        SmartDashboard.putNumber(getName() + "EC Setpoint", _elevatorController.getSetpoint());

    }
    //#endregion

    //#region Commands

    public Command elevatorDefaultCommand(DoubleSupplier elevatorManualControl) {
        return this.run(() -> manageElevatorControl(elevatorManualControl.getAsDouble()));
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
