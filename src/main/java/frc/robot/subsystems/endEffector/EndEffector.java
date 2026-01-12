package frc.robot.subsystems.endEffector;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.SubsystemControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.SuperStructure;
import frc.robot.subsystems.elevator.ElevatorPosition;

public class EndEffector extends SubsystemBase {
    public IEndEffector EndEffectorIO;
    public PIDController PIDController;

    public EndEffector(boolean isReal) {
        setName("End Effector");
        EndEffectorIO = isReal
                ? new EndEffectorReal()
                : new EndEffectorSim();

        PIDController = EndEffectorMap.WristPID.createPIDController(0.02);
        SmartDashboard.putData(PIDController);
    }

    //#region Control

    public void seekWristAnglePID(boolean inDangerZone) {
        SmartDashboard.putBoolean(getName() + "/InDangerZone", inDangerZone);

        var safeAngle = EndEffectorMap.ElevatorDangerZoneWristAngleLookup.get(SuperStructure.Elevator.DistanceMeters);
        SmartDashboard.putNumber(getName() + "/dangerZoneCalculatedSafeAngle", safeAngle);

        SmartDashboard.putNumber(getName() + "/PID-Setpoint", PIDController.getSetpoint());
        double pid = inDangerZone
                ? PIDController.calculate(SuperStructure.EndEffector.EndEffectorAngleDegrees, 0) // Override internal setpoint
                : PIDController.calculate(SuperStructure.EndEffector.EndEffectorAngleDegrees);

        SmartDashboard.putNumber(getName() + "/PID-Raw", pid);
        pid = MathUtil.clamp(pid, -EndEffectorMap.WristMaxOutput, EndEffectorMap.WristMaxOutput);

        SmartDashboard.putNumber(getName() + "/PID-FinalOutput", pid);
        EndEffectorIO.setWristSpeed(pid);
    }

    public void runWristManual(double speed) {
        // Invert and scale to max output
        double manualMotorControl = -speed * EndEffectorMap.WristMaxOutput;

        // Artifically limit the wrist angle
        if (SuperStructure.EndEffector.EndEffectorAngleDegrees <= EndEffectorMap.WristMinAngle) {
            manualMotorControl = Math.max(manualMotorControl, 0);
        } else if (SuperStructure.EndEffector.EndEffectorAngleDegrees >= EndEffectorMap.WristMaxAngle) {
            manualMotorControl = Math.min(manualMotorControl, 0);
        }

        EndEffectorIO.setWristSpeed(manualMotorControl);
    }

    /**
     * Sets the wrist setpoint to the given angle, or the max angle if the given angle is greater
     * than the max angle.
     * @param angle Desired angle
     */
    public void setWristSetpoint(double angle) {
        PIDController.setSetpoint(Math.min(EndEffectorMap.WristMaxAngle, angle));
    }

    //#endregion

    @Override
    public void periodic() {
        EndEffectorIO.updateInputs(SuperStructure.EndEffector);
        Logger.processInputs(getName(), SuperStructure.EndEffector);

        // Check if the elevator and wrist are in a safe state for manual control
        boolean inDangerZone = SuperStructure.Elevator.DistanceMeters <= EndEffectorMap.LowerElevatorSafetyLimit;
        boolean wristAngleSafe = SuperStructure.EndEffector.EndEffectorAngleDegrees <= EndEffectorMap.WristMaxManuallyControllableAngle;
        boolean manualControlNotSafe = inDangerZone || !wristAngleSafe;

        // Automatically switch to closed-loop if manual control is unsafe
        if (manualControlNotSafe && SuperStructure.EndEffector.ControlMode == SubsystemControlMode.ManuallyControlled) {
            SuperStructure.EndEffector.ControlMode = SubsystemControlMode.ClosedLoopControlled;
        }

        if (SuperStructure.EndEffector.ControlMode == SubsystemControlMode.ManuallyControlled && !inDangerZone) {
            runWristManual(SuperStructure.EndEffector.ManualControlSpeed);
        } else {
            seekWristAnglePID(inDangerZone);
        }
    }

    public boolean wristAtSetpoint() {
        return PIDController.atSetpoint();
    }

    /**
     * Default command which runs the wrist PID and runs the intake based on two buttons
     * @param runIntakeIn
     * @param runIntakeOut
     */
    public Command defaultCommand(BooleanSupplier runIntakeOut, DoubleSupplier wristManualControl) {
        return this.run(() -> {
            // Driver is trying to eject && InTeleop or use instance variable (used in automation)
            if ((runIntakeOut.getAsBoolean() && DriverStation.isTeleopEnabled()) ||
                    SuperStructure.EndEffector.ManuallyEjecting) {
                EndEffectorIO.setIntakeSpeed(EndEffectorMap.EjectSpeed);
            } else {
                EndEffectorIO.stopIntakeMotor();
            }

            SuperStructure.EndEffector.ManualControlSpeed = MathUtil
                    .applyDeadband(wristManualControl.getAsDouble(), 0.05);
            if (Math.abs(SuperStructure.EndEffector.ManualControlSpeed) > 0.05) {
                SuperStructure.EndEffector.ControlMode = SubsystemControlMode.ManuallyControlled;
            } else {
                SuperStructure.EndEffector.ControlMode = SubsystemControlMode.ClosedLoopControlled;
                // When switching from manual to closed loop, set the setpoint to the current angle
                setWristSetpoint(SuperStructure.EndEffector.EndEffectorAngleDegrees);
            }
        });
    }

    /**
     * Sets the wrist setpoint and disables manual control
     * @param angle Desired angle
     */
    public Command setWristSetpointCommand(double angle) {
        return disableWristManualControlCommand()
                .andThen(() -> setWristSetpoint(angle));
    }

    /**
    
     * Sets the wrist setpoint
     * @param position Elevator position
     */
    public Command setWristSetpointCommand(ElevatorPosition position) {
        return setWristSetpointCommand(EndEffectorMap.LiftHeightWristAngleMap.get(position));
    }

    public Command wristManualControlCommand(double speed) {
        return Commands.runOnce(() -> EndEffectorIO.setWristSpeed(speed));
    }

    public Command disableWristManualControlCommand() {
        return Commands.runOnce(() -> {
            SuperStructure.EndEffector.ManualControlSpeed = 0;
            SuperStructure.EndEffector.ControlMode = SubsystemControlMode.ClosedLoopControlled;
        });
    }

    public Command stopIntakeMotorCommand() {
        return Commands.runOnce(() -> EndEffectorIO.stopIntakeMotor());
    }

    public Command stopWristMotorCommand() {
        return Commands.runOnce(() -> EndEffectorIO.stopWristMotor());
    }

    public Command stopBothMotorsCommand() {
        return this.runOnce(() -> EndEffectorIO.stopMotors());
    }

    public Command enableIntakeCommand() {
        return this.runOnce(() -> {
            EndEffectorIO.setIntakeSpeed(EndEffectorMap.IntakeSpeed);
            SuperStructure.EndEffector.ManuallyEjecting = false;
        });
    }

    public Command enableEjectCommand() {
        return this.runOnce(() -> {
            EndEffectorIO.setIntakeSpeed(EndEffectorMap.EjectSpeed);
            SuperStructure.EndEffector.ManuallyEjecting = true;
        });
    }

    public Command disableEjectCommand() {
        return this.runOnce(() -> {
            EndEffectorIO.stopIntakeMotor();
            SuperStructure.EndEffector.ManuallyEjecting = false;
        });
    }

    public Command scoreCoral() {
        return Commands.waitUntil(this::wristAtSetpoint)
                .andThen(enableEjectCommand()) // eject until limit switch is let go (or times out)
                .until(() -> !SuperStructure.EndEffector.CoralLimitSwitchState).withTimeout(2)
                .andThen(Commands.waitSeconds(0.75)) // slight delay to allow the coral to begin ejecting
                .andThen(setWristSetpointCommand(-30)) // set the angle up while ejecting
                .andThen(Commands.waitSeconds(0.75))
                .andThen(disableEjectCommand());
    }

    public Command pickupCoral() {
        return enableIntakeCommand()
                .andThen(Commands.waitUntil(() -> SuperStructure.EndEffector.CoralLimitSwitchState))
                .andThen(disableEjectCommand())
                .andThen(setWristSetpointCommand(0))
                .andThen(Commands.print(">> INTAKE: Coral picked up"));
    }
}