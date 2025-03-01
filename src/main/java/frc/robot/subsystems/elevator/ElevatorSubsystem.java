
package frc.robot.subsystems.elevator;

import java.util.Map;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.dashboard.SendableButton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
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
        public static final int leftElevatorMotorCANID = 20;
        public static final int rightElevatorMotorCANID = 21;
        public static final int topLimitSwitchChannel = 0;
        public static final int bottomLimitSwitchChannel = 1;
        public static final int maxPercentOutput = 1;

        public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(0.01, 0, 0, 0, 0.01, 0.01,
                0.01);
        public static final double FeedForwardKg = 0.0;

        // TODO: Measure
        public static final double OutputSprocketDiameterMeters = 0;
        public static final double GearRatio = 0;
    }

    private SendableButton _stopMotorsButton;
    private SendableButton _sourcePosButton;
    private SendableButton _troughPosButton;
    private SendableButton _lowPosButton;
    private SendableButton _midPosButton;
    private SendableButton _highPosButton;
    private double elevatorSetpoint = 0;

    public enum ElevatorPosition {
        kSource,
        kTrough,
        kLow,
        kMid,
        kHigh
    }

    private Map<ElevatorPosition, Double> _positionMap = Map.of(
            ElevatorPosition.kSource, 0.0,
            ElevatorPosition.kTrough, 1.0,
            ElevatorPosition.kLow, 2.0,
            ElevatorPosition.kMid, 3.0,
            ElevatorPosition.kHigh, 4.0);

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
                ElevatorMap.PositionPID.kV);

        SmartDashboard.putData(_positionPidController);

        // TODO: Remove when no longer needed
        _sysId = new SysIdRoutine(
                // Ramp up at 1 volt per second for quasistatic tests, step at 2 volts in
                // dynamic tests, run for 13 seconds.
                new SysIdRoutine.Config(Units.Volts.of(2).per(Units.Second), Units.Volts.of(4),
                        Units.Seconds.of(7)),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        this::setMotorVoltages,
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being characterized.
                        this::logSysIdFrame,
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in WPILog with this subsystem's name
                        this));

        _stopMotorsButton = new SendableButton("Stop Elevator Motors", () -> stopMotorsCommand());
        Container.TestDashboardSection.putData("Elevator/Stop Motors", _stopMotorsButton);
        _sourcePosButton = new SendableButton("Source Position",
                () -> goToElevatorPositionCommand(ElevatorPosition.kSource));
        Container.TestDashboardSection.putData("Elevator/Source Position", _sourcePosButton);
        _troughPosButton = new SendableButton("Trough Position",
                () -> goToElevatorPositionCommand(ElevatorPosition.kTrough));
        Container.TestDashboardSection.putData("Elevator/Trough Position", _troughPosButton);
        _lowPosButton = new SendableButton("Low Position", () -> goToElevatorPositionCommand(ElevatorPosition.kLow));
        Container.TestDashboardSection.putData("Elevator/Low Position", _lowPosButton);
        _midPosButton = new SendableButton("Middle Position", () -> goToElevatorPositionCommand(ElevatorPosition.kMid));
        Container.TestDashboardSection.putData("Elevator/Middle Position", _midPosButton);
        _highPosButton = new SendableButton("High Position", () -> goToElevatorPositionCommand(ElevatorPosition.kHigh));
        Container.TestDashboardSection.putData("Elevator/High Position", _highPosButton);

    }

    //#region Control

    private void setPositionSetpoint(ElevatorPosition pos) {

    }

    private void updateMotorSpeeds() {
        // TODO: check this logic
        var pid = _positionPidController.calculate(_inputs.ElevatorDistanceMeters);
        var ff = _positionFeedforward.calculate(pid);
        var finalOutput = MathUtil.clamp(pid + ff, -ElevatorMap.maxPercentOutput, ElevatorMap.maxPercentOutput);

        if (_inputs.TopLimitSwitch && finalOutput > 0) {
            finalOutput = MathUtil.clamp(finalOutput, -ElevatorMap.maxPercentOutput, 0);

        } else if (_inputs.BottomLimitSwitch && finalOutput < 0) {
            finalOutput = MathUtil.clamp(finalOutput, 0, ElevatorMap.maxPercentOutput);

        }
        _elevatorIO.setMotorSpeeds(finalOutput);
    }

    private void setMotorVoltages(Voltage volts) {
        _elevatorIO.setMotorVoltages(volts.magnitude());

    }

    @Override
    public void periodic() {
        _elevatorIO.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
        Logger.recordOutput("Elevator/ElevatorSetpoint", elevatorSetpoint);
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
        return this.run(this::updateMotorSpeeds);
    }

    public Command goToElevatorPositionCommand(ElevatorPosition pos) {
        return Commands.runOnce(() -> {
            elevatorSetpoint = _positionMap.get(pos);
            _positionPidController.setSetpoint(elevatorSetpoint);
            System.out.println(elevatorSetpoint);
        });
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

    //#endregion
}
