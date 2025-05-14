package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Hertz;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Container;
import frc.robot.Robot;
import frc.robot.SuperStructure;
import frc.robot.subsystems.elevator.ElevatorMap;

public class Climber extends SubsystemBase {
    public enum ClimberPosition {
        /** The starting position of the climber (Up) */
        IN,
        /** The ending position of the climber (Down) */
        OUT
    }

    public enum HooksPosition {
        /** The servo arms will be open */
        OPEN,
        /** The servo arms will be closed) */
        CLOSED
    }

    private IClimber _climber;

    private BooleanEvent _positionResetEvent;
    private Trigger _setClimberOutTimedTrigger;

    private LEDPattern _winchRetractPattern = LEDPattern
            .gradient(GradientType.kContinuous, Color.kBlack, Color.kDarkGoldenrod)
            .scrollAtRelativeSpeed(Hertz.of(2))
            .reversed();
    private LEDPattern _hooksRunningOutPattern = _winchRetractPattern
            .overlayOn(_winchRetractPattern.reversed());

    public Climber(Boolean isReal) {
        setName("Climber");
        _climber = isReal
                ? new ClimberReal()
                : new ClimberSim();

        setupTriggersAndEvents();
    }

    /**
     * Sets up the automatic triggers and events
     */
    private void setupTriggersAndEvents() {
        _positionResetEvent = new BooleanEvent(Robot.EventLoop, () -> SuperStructure.ClimberState.WinchOuterLimitSwitch)
                .debounce(ElevatorMap.BottomLimitResetDebounceSeconds)
                .rising();
        _positionResetEvent.ifHigh(_climber::resetClimberAngle);

        _setClimberOutTimedTrigger = new Trigger(Robot.EventLoop,
                () -> DriverStation.getMatchTime() <= 50
                        && DriverStation.isTeleopEnabled()
                        && DriverStation.isFMSAttached());
        _setClimberOutTimedTrigger.onTrue(extendWinchAuto());
    }

    //#region Control Methods

    /**
     * Sets the speed of the hooks motor. Positive is closing, negative is opening
     * @param speed
     */
    private void setHooksSpeed(double speed) {
        if ((speed > 0 && SuperStructure.ClimberState.HooksClosedLimitSwitch) || (speed < 0 && SuperStructure.ClimberState.HooksOpenLimitSwitch)) {
            _climber.stopHooksMotors();
            Logger.recordOutput(getName() + "/hooks-speed", 0);
            return;
        }

        _climber.setHookMotorSpeed(speed);
        Logger.recordOutput(getName() + "/hooks-speed", speed);
    }

    /**
     * Sets the speed of the climber motor. Positive is winching inward, negative is letting it out
     * @param speed
     */
    private void setWinchSpeed(double speed) {
        if (speed > 0) {
            if (SuperStructure.ClimberState.WinchInnerLimitSwitch) {
                _climber.stopWinchMotors();
                return;
            }
        } else if (speed < 0 && SuperStructure.ClimberState.WinchOuterLimitSwitch) {
            _climber.stopWinchMotors();
            return;
        }

        _climber.setWinchSpeed(speed);
        Logger.recordOutput(getName() + "/winch-speed", speed);
    }

    //#endregion

    @Override
    public void periodic() {
        _climber.updateInputs(SuperStructure.ClimberState);
        Logger.processInputs(getName(), SuperStructure.ClimberState);
    }

    //#region Commands

    /**
     * Runs the winch motor in the extending direction
     * @return
     */
    public Command extendWinchCommand() {
        return this.run(() -> setWinchSpeed(-ClimberMap.WinchInSpeed));
    }

    /**
     * Runs the winch motor in the retracting direction
     * @return
     */
    public Command retractWinchCommand() {
        return this.run(() -> setWinchSpeed(ClimberMap.WinchInSpeed));
    }

    /**
     * Runs the hooks motor in the closing direction
     * @return
     */
    public Command runHooksClosedCommand() {
        return this.run(() -> setHooksSpeed(ClimberMap.HookCloseSpeed));
    }

    /**
     * Runs the hooks motor in the opening direction
     * @return
     */
    public Command runHooksOpenCommand() {
        return this.run(() -> setHooksSpeed(-ClimberMap.HookCloseSpeed));
    }

    /**
     * Template for running a movement until a limit switch is hit, then stopping that movement
     * @param moveCommand
     * @param limitSwitch
     * @param stopCommand
     * @return
     */
    private Command runUntilLimitSwitch(Command moveCommand, BooleanSupplier limitSwitch, Command stopCommand) {
        return moveCommand.until(limitSwitch).andThen(stopCommand).finallyDo(() -> {
            _climber.stopHooksMotors();
            _climber.stopWinchMotors();
        }).withTimeout(5);
    }

    /**
     * Automatically closes the hooks until the limit switch is hit
     * @return
     */
    public Command setHooksClosedAuto() {
        return runUntilLimitSwitch(runHooksClosedCommand(), () -> SuperStructure.ClimberState.HooksClosedLimitSwitch,
                stopHooksMotorCommand());
    }

    /**
     * Automatically opens the hooks until the limit switch is hit
     * @return
     */
    public Command setHooksOpenAuto() {
        var movementCommand = runHooksOpenCommand()
                .alongWith(Container.LEDs.setAllSectionPatternsCommand(_hooksRunningOutPattern));
        var stopMovementCommand = stopHooksMotorCommand()
                .alongWith(Container.LEDs.setAllSectionPatternsCommand(LEDPattern.solid(Color.kGreen)));

        return runUntilLimitSwitch(movementCommand, () -> SuperStructure.ClimberState.HooksOpenLimitSwitch, stopMovementCommand);
    }

    /**
     * Automatically extends the winch until the outer limit switch is hit
     * @return
     */
    public Command extendWinchAuto() {
        return runUntilLimitSwitch(extendWinchCommand(), () -> SuperStructure.ClimberState.WinchOuterLimitSwitch, stopWinchMotorsCommand());
    }

    /**
     * Automatically retracts the winch until the inner limit switch is hit
     * @return
     */
    public Command retractWinchAuto() {
        var movementCommand = retractWinchCommand()
                .alongWith(Container.LEDs.setAllSectionPatternsCommand(_winchRetractPattern));
        var stopMovementCommand = stopWinchMotorsCommand()
                .alongWith(Container.LEDs.setAllSectionPatternsCommand(LEDPattern.solid(Color.kGreen)));

        return runUntilLimitSwitch(movementCommand,
                () -> SuperStructure.ClimberState.climberShaftRotations > ClimberMap.FullyClimbedOutputRotations
                        || SuperStructure.ClimberState.WinchInnerLimitSwitch,
                stopMovementCommand);
    }

    /**
     * Stops the climbing motors
     * @return
     */
    public Command stopWinchMotorsCommand() {
        return this.runOnce(_climber::stopWinchMotors);
    }

    /**
     * Stops the hooks motor
     * @return
     */
    public Command stopHooksMotorCommand() {
        return this.runOnce(_climber::stopHooksMotors);
    }

    /**
     * Stops all motors
     * @return
     */
    public Command stopAllMotors() {
        return stopWinchMotorsCommand()
                .andThen(stopHooksMotorCommand());
    }

    //#endregion
}