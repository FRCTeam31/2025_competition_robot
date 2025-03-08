
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.dashboard.SendableButton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Container;

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
    }

    public enum ElevatorPosition {
        kSource,
        kTrough,
        kLow,
        kMid,
        kHigh
    }

    private Map<ElevatorPosition, Double> _positionMap = Map.of(
            ElevatorPosition.kSource, 0.0,
            ElevatorPosition.kTrough, -0.63,
            ElevatorPosition.kLow, 0.3,
            ElevatorPosition.kMid, 0.45,
            ElevatorPosition.kHigh, 0.63);

    private ElevatorInputsAutoLogged _inputs = new ElevatorInputsAutoLogged();
    private IElevator _elevatorIO;
    private PIDController _positionPidController;
    private ElevatorFeedforward _positionFeedforward;

    private SysIdRoutine _sysId;

    public ElevatorSubsystem(boolean isReal) {
        setName("Elevator");

        _elevatorIO = isReal
                ? new ElevatorReal()
                : new ElevatorSim();

        _positionPidController = ElevatorMap.PositionPID.createPIDController(0.02);
        _positionFeedforward = new ElevatorFeedforward(ElevatorMap.PositionPID.kS, ElevatorMap.FeedForwardKg,
                ElevatorMap.PositionPID.kV, ElevatorMap.PositionPID.kA);

        SmartDashboard.putData(_positionPidController);

        // TODO: Remove when no longer needed
        _sysId = new SysIdRoutine(
                // Ramp up at 1 volt per second for quasistatic tests, step at 2 volts in
                // dynamic tests, run for 13 seconds.
                new SysIdRoutine.Config(Units.Volts.of(2).per(Units.Second), Units.Volts.of(3),
                        Units.Seconds.of(4)),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        this::setMotorVoltages,
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being characterized.
                        this::logSysIdFrame,
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in WPILog with this subsystem's name
                        this));

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
        if (finalOutput > 0 && _inputs.ElevatorDistanceMeters > (ElevatorMap.MaxElevatorHeight
                - (ElevatorMap.MaxElevatorHeight * 0.15))) {
            finalOutput = MathUtil.clamp(finalOutput, ElevatorMap.MaxSpeedCoefficient,
                    ElevatorMap.MaxUpSpeedCoefficient);
        }

        SmartDashboard.putNumber(getName() + "/finalOutput", finalOutput);
        SmartDashboard.putNumber(getName() + "/PID", pid);
        SmartDashboard.putNumber(getName() + "/feedForward", ff);
        SmartDashboard.putNumber(getName() + "/setpoint", _positionPidController.getSetpoint());

        setMotorSpeedsWithLimitSwitches(finalOutput);
    }

    private void setMotorVoltages(Voltage volts) {
        var vMag = volts.magnitude();
        if (_inputs.TopLimitSwitch && vMag > 0) {
            vMag = MathUtil.clamp(vMag, -ElevatorMap.maxVoltage * ElevatorMap.MaxSpeedCoefficient, 0);
        } else if (_inputs.BottomLimitSwitch && vMag < 0) {
            vMag = MathUtil.clamp(vMag, 0, ElevatorMap.maxVoltage * ElevatorMap.MaxSpeedCoefficient);
        }

        SmartDashboard.putNumber(getName() + "/Output-V", vMag);
        _elevatorIO.setMotorVoltages(vMag);
    }

    @Override
    public void periodic() {
        _elevatorIO.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
        // Logger.recordOutput("Elevator/ElevatorSetpoint", elevatorSetpoint);
        if (_inputs.BottomLimitSwitch) {
            _elevatorIO.resetElevatorPosition();
        }
    }

    /**
    * Logs a sysid frame for the FL module
    * @param log
    */
    public void logSysIdFrame(SysIdRoutineLog log) {
        // Record a frame. Since these share an encoder, we consider
        // the entire group to be one motor.
        log.motor("Elevator-Output")
                .voltage(Units.Volts.of(_inputs.MotorVoltage)) // measured motor voltage
                .linearPosition(Units.Meters.of(_inputs.ElevatorDistanceMeters)) // distance in meters
                .linearVelocity(Units.MetersPerSecond.of(_inputs.ElevatorSpeedMetersPerSecond)); // speed in meters per second
    }

    //#endregion

    //#region Commands

    public Command runElevatorAutomaticSeekCommand() {
        return this.run(this::updateMotorSpeedsWithPID);
    }

    public Command goToElevatorPositionCommand(ElevatorPosition pos) {
        return Commands.runOnce(() -> {
            _positionPidController.setSetpoint(_positionMap.get(pos));
        });
    }

    public Command goToElevatorBottomCommand() {
        return Commands.runOnce(() -> {
            _positionPidController.setSetpoint(-ElevatorMap.MaxElevatorHeight);
        }).andThen(Commands.waitUntil(() -> _inputs.BottomLimitSwitch)).andThen(Commands.runOnce(() -> {
            _positionPidController.setSetpoint(0);
        }));
    }

    public Command stopMotorsCommand() {
        return this.runOnce(_elevatorIO::stopMotors);
    }

    // TODO: Remove when no longer needed
    public Command runSysIdQuasistaticRoutineCommand(Direction dir) {
        return _sysId.quasistatic(dir);
    }

    // TODO: Remove when no longer needed
    public Command runSysIdDynamicRoutineCommand(Direction dir) {
        return _sysId.dynamic(dir);
    }

    public Map<String, Command> elevatorNamedCommands() {
        return Map.of(
                "Stop Elevator Motors Command", stopMotorsCommand(),
                "Elevator High Position Command", goToElevatorPositionCommand(ElevatorPosition.kHigh),
                "Elevator Middle Position Command", goToElevatorPositionCommand(ElevatorPosition.kMid),
                "Elevator Low Position Command", goToElevatorPositionCommand(ElevatorPosition.kLow),
                "Elevator Trough Position Command", goToElevatorPositionCommand(ElevatorPosition.kTrough),
                "Elevator Source Position Command", goToElevatorPositionCommand(ElevatorPosition.kSource));
    }

    //#endregion
}
