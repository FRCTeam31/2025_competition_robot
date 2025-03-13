
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
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
        public static final double MaxDownSpeedCoefficient = -0.25;
        public static final double MaxUpSpeedCoefficient = 0.25;
        // public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(0.043861, 0, 0, 0,
        //         0.28922,
        //         0.024268,
        //         0.29376);

        public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(4, 0, 0.04, 0,
                0.028922,
                0.024268,
                0.029376);
        public static final double FeedForwardKg = 0.16733 / 2;
        public static final double OutputSprocketDiameterMeters = Units.Millimeters.of(32.2).in(Meters);
        public static final double GearRatio = 16;
        public static final double maxVoltage = 12;
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
            ElevatorPosition.kHigh, 0.654);

    private ElevatorInputsAutoLogged _inputs = new ElevatorInputsAutoLogged();
    private IElevator _elevatorIO;
    private PIDController _positionPidController;
    private ElevatorFeedforward _positionFeedforward;

    public ElevatorSubsystem(boolean isReal) {
        setName("Elevator");

        _elevatorIO = isReal
                ? new ElevatorReal()
                : new ElevatorSim();

        _positionPidController = ElevatorMap.PositionPID.createPIDController(0.02);
        _positionFeedforward = new ElevatorFeedforward(ElevatorMap.PositionPID.kS, ElevatorMap.FeedForwardKg,
                ElevatorMap.PositionPID.kV, ElevatorMap.PositionPID.kA);

        SmartDashboard.putData(_positionPidController);

        // _stopMotorsButton = new SendableButton("Stop Elevator Motors", () -> stopMotorsCommand());
        // Container.TestDashboardSection.putData("Elevator/Stop Motors", _stopMotorsButton);
        // _sourcePosButton = new SendableButton("Source Position",
        //         () -> goToElevatorPositionCommand(ElevatorPosition.kSource));
        // Container.TestDashboardSection.putData("Elevator/Source Position", _sourcePosButton);
        // _troughPosButton = new SendableButton("Trough Position",
        //         () -> goToElevatorPositionCommand(ElevatorPosition.kTrough));
        // Container.TestDashboardSection.putData("Elevator/Trough Position", _troughPosButton);
        // _lowPosButton = new SendableButton("Low Position", () -> goToElevatorPositionCommand(ElevatorPosition.kLow));
        // Container.TestDashboardSection.putData("Elevator/Low Position", _lowPosButton);
        // _midPosButton = new SendableButton("Middle Position", () -> goToElevatorPositionCommand(ElevatorPosition.kMid));
        // Container.TestDashboardSection.putData("Elevator/Middle Position", _midPosButton);
        // _highPosButton = new SendableButton("High Position", () -> goToElevatorPositionCommand(ElevatorPosition.kHigh));
        // Container.TestDashboardSection.putData("Elevator/High Position", _highPosButton);

    }

    public double getElevatorPositionMeters() {
        return _inputs.ElevatorDistanceMeters;
    }

    public double getElevatorPositionPercent() {
        return getElevatorPositionMeters() / ElevatorMap.MaxElevatorHeight;
    }

    //#region Control

    public void setMotorSpeedsWithLimitSwitches(double finalOutput) {
        if (_inputs.TopLimitSwitch && finalOutput > 0) {
            finalOutput = MathUtil.clamp(finalOutput, -ElevatorMap.MaxPercentOutput, 0);
        } else if (_inputs.BottomLimitSwitch && finalOutput < 0) {
            finalOutput = MathUtil.clamp(finalOutput, 0, ElevatorMap.MaxPercentOutput);
        }

        SmartDashboard.putNumber(getName() + "/Output-PIDFF", finalOutput);
        _elevatorIO.setMotorSpeeds(finalOutput);
    }

    private void updateMotorSpeedsWithPID() {
        var pid = _positionPidController.calculate(_inputs.ElevatorDistanceMeters);
        var desiredVelocity = _positionPidController.getError() * ElevatorMap.MaxSpeedCoefficient;
        var ff = _positionFeedforward.calculate(desiredVelocity);
        var finalOutput = MathUtil.clamp(pid + ff, -ElevatorMap.MaxSpeedCoefficient, ElevatorMap.MaxSpeedCoefficient);

        if (finalOutput < 0 && _inputs.ElevatorDistanceMeters < (ElevatorMap.MaxElevatorHeight * 0.15)) {
            finalOutput = MathUtil.clamp(finalOutput, ElevatorMap.MaxDownSpeedCoefficient,
                    ElevatorMap.MaxSpeedCoefficient);
        }

        // Within 5% of max height, reduce speed
        finalOutput = getScaledSpeed(finalOutput, _inputs.ElevatorDistanceMeters);

        SmartDashboard.putNumber(getName() + "/finalOutput", finalOutput);
        SmartDashboard.putNumber(getName() + "/PID", pid);
        SmartDashboard.putNumber(getName() + "/feedForward", ff);
        SmartDashboard.putNumber(getName() + "/setpoint", _positionPidController.getSetpoint());

        setMotorSpeedsWithLimitSwitches(finalOutput);
    }

    private double getScaledSpeed(double outputSpeed, double currentPosition) {
        var lowPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight * 0.15;
        var highPositionScaleDownThreshold = ElevatorMap.MaxElevatorHeight - lowPositionScaleDownThreshold;

        var speedScalingFactor = 1.0;
        if (currentPosition <= lowPositionScaleDownThreshold && outputSpeed < 0) {
            var scale = currentPosition / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.1, scale);
        } else if (currentPosition >= highPositionScaleDownThreshold && outputSpeed > 0) {
            var scale = (ElevatorMap.MaxElevatorHeight - currentPosition) / lowPositionScaleDownThreshold;
            speedScalingFactor = Math.max(0.1, scale);
        }

        return outputSpeed * speedScalingFactor;
    }

    // private void setMotorVoltages(Voltage volts) {
    //     var vMag = volts.magnitude();
    //     if (_inputs.TopLimitSwitch && vMag > 0) {
    //         vMag = MathUtil.clamp(vMag, -ElevatorMap.maxVoltage * ElevatorMap.MaxSpeedCoefficient, 0);
    //     } else if (_inputs.BottomLimitSwitch && vMag < 0) {
    //         vMag = MathUtil.clamp(vMag, 0, ElevatorMap.maxVoltage * ElevatorMap.MaxSpeedCoefficient);
    //     }

    //     SmartDashboard.putNumber(getName() + "/Output-V", vMag);
    //     _elevatorIO.setMotorVoltages(vMag);
    // }

    @Override
    public void periodic() {
        _elevatorIO.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
        // Logger.recordOutput("Elevator/ElevatorSetpoint", elevatorSetpoint);

    }

    //#endregion

    //#region Commands

    public Command runElevatorWithController(DoubleSupplier dSup) {
        return this.run(() -> setMotorSpeedsWithLimitSwitches(
                MathUtil.clamp(dSup.getAsDouble(), -ElevatorMap.MaxSpeedCoefficient, ElevatorMap.MaxSpeedCoefficient)));
    }

    public Command runElevatorAutomaticSeekCommand() {
        return this.run(this::updateMotorSpeedsWithPID);
    }

    public Command goToElevatorPositionCommand(ElevatorPosition pos) {
        return Commands.runOnce(() -> {
            _positionPidController.setSetpoint(_positionMap.get(pos));
        });
    }

    public Command goToElevatorBottomCommand() {
        return Commands
                .run(() -> _elevatorIO.setMotorSpeeds(ElevatorMap.MaxDownSpeedCoefficient))
                .until(() -> _inputs.BottomLimitSwitch)
                .withTimeout(7)
                .andThen(Commands.runOnce(() -> {
                    _elevatorIO.stopMotors();
                    _positionPidController.setSetpoint(0);
                }));
    }

    public Command stopMotorsCommand() {
        return this.runOnce(_elevatorIO::stopMotors);
    }

    public Map<String, Command> elevatorNamedCommands() {
        return Map.of(
                "Stop Elevator Motors Command", stopMotorsCommand(),
                "Elevator High Position Command", goToElevatorPositionCommand(ElevatorPosition.kHigh),
                "Elevator Middle Position Command", goToElevatorPositionCommand(ElevatorPosition.kMid),
                "Elevator Low Position Command", goToElevatorPositionCommand(ElevatorPosition.kLow),
                "Elevator Trough Position Command", goToElevatorPositionCommand(ElevatorPosition.kTrough),
                "Elevator Source Position Command", goToElevatorPositionCommand(ElevatorPosition.kAbsoluteMinimum));
    }

    //#endregion
}
