package frc.robot.subsystems.endEffector;

import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

    public class EndEffectorMap {
        public static final byte IntakeMotorCanID = 127;
        public static final byte WristMotorCanID = 127;
        public static final byte LimitSwitchCanID = 0;
        public static final double EjectSpeed = 0.5;
        public static final double IntakeSpeed = -0.5;
        public static final byte IntakeCurrentLimit = 20;
        public static final byte WristCurrentLimit = 30;

        public static final ExtendedPIDConstants WristPID = new ExtendedPIDConstants(0, 0, 0);

        public static final double GearRatio = 5;

        public static final double MaxWristAngle = 135;
        public static final double MinWristAngle = 0;

        // Wrist Constants
        public static final double WristUp = 0;
        public static final double WristSource = 45;
        public static final double WristReef = 135;
        public static final double WristMaxOutput = 0.25;
    }

    private IEndEffector _endEffector;
    private PIDController _wristPID;

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    // TODO: Add a system to set the wrist angle to a predefined setpoint. Look at the system used in the elevator subsystem for reference. We will most likely use the same enums used in the elevator subsystem so it is easier to implement the auto rotating system later. For now, create a temporary enum.

    public EndEffectorSubsystem(boolean isReal) {
        setName("End Effector");
        _endEffector = isReal ? new EndEffectorReal() : new EndEffectorSim();
        _wristPID = EndEffectorMap.WristPID.createPIDController(0.02);
    }

    public void seekWristAngle() {
        double pid = _wristPID.calculate(_inputs.EndEffectorAngleDegrees);
        SmartDashboard.putNumber(getName() + "/WristPID", pid);

        pid = MathUtil.clamp(pid, -EndEffectorMap.WristMaxOutput, EndEffectorMap.WristMaxOutput);

        Logger.recordOutput(getName() + "/WristPID-FinalOutput", pid);
        // _endEffector.setWristSpeed(pid);
    }

    @Override
    public void periodic() {
        _endEffector.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
    }

    /**
     * Default command which runs the wrist PID and runs the intake based on two buttons
     * @param runIntakeIn
     * @param runIntakeOut
     */
    public Command defaultCommand(BooleanSupplier runIntakeIn, BooleanSupplier runIntakeOut) {
        return this.run(() -> {
            if (runIntakeIn.getAsBoolean()) {
                _endEffector.setIntakeSpeed(EndEffectorMap.IntakeSpeed);
            } else if (runIntakeOut.getAsBoolean()) {
                _endEffector.setIntakeSpeed(EndEffectorMap.EjectSpeed);
            } else {
                _endEffector.stopIntakeMotor();
            }

            seekWristAngle();
        });
    }

    /**
     * Sets the wrist setpoint
     * @param angle Desired angle
     */
    public Command setWristSetpointCommand(double angle) {
        return Commands.runOnce(() -> _wristPID.setSetpoint(angle));
    }

    public Command stopIntakeMotorCommand() {
        return Commands.runOnce(() -> _endEffector.stopIntakeMotor());
    }

    public Command stopWristMotorCommand() {
        return Commands.runOnce(() -> _endEffector.stopWristMotor());
    }

    public Command stopBothMotorsCommand() {
        return Commands.runOnce(() -> _endEffector.stopMotors());
    }

    public Command setIntakeSpeedCommand(double speed) {
        return Commands.runOnce(() -> _endEffector.setIntakeSpeed(speed));
    }

    // TODO: Add more named commands for running the intake and rotating the wrist to predefined setpoints (See above todo)
    public Map<String, Command> getNamedCommands() {
        return Map.of("Stop Intake", stopIntakeMotorCommand(), "Stop Both Motors", stopBothMotorsCommand(),
                "Intake Coral",
                setIntakeSpeedCommand(EndEffectorMap.IntakeSpeed), "Eject Coral",
                setIntakeSpeedCommand(EndEffectorMap.EjectSpeed));
    }

}