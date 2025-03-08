package frc.robot.subsystems.endEffector;

import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;

import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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

        // TODO: Verify wether this is the gear ratio for the wrist or the intake, it is used for both in sim
        public static final double GearRatio = 5;

        public static final double MaxWristAngle = 135;
        public static final double MinWristAngle = 0;

    }

    private IEndEffector _endEffector;

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    // TODO: Add a system to set the wrist angle to a predefined setpoint. Look at the system used in the elevator subsystem for reference. We will most likely use the same enums used in the elevator subsystem so it is easier to implement the auto rotating system later. For now, create a temporary enum.

    public EndEffectorSubsystem(boolean isReal) {
        setName("End Effector");
        _endEffector = isReal ? new EndEffectorReal() : new EndEffectorSim();
    }

    @Override
    public void periodic() {
        _endEffector.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
    }

    public Command setEndEffectorWristAngle(Rotation2d angle) {
        return this.run(() -> {
            _endEffector.setWristAngle(angle);
        });
    }

    public Command runEndEffectorIntakeCommand(double speed) {
        return this.runOnce(() -> {
            _endEffector.setIntakeMotorSpeed(speed);
        });
    }

    public Command stopEndEffectorIntakeCommand() {
        return this.runOnce(() -> {
            _endEffector.stopIntakeMotor();
        });
    }

    // TODO: Add more named commands for running the intake and rotating the wrist to predefined setpoints (See above todo)
    public Map<String, Command> getNamedCommands() {
        return Map.of("Stop Intake", stopEndEffectorIntakeCommand(), "Intake Coral",
                runEndEffectorIntakeCommand(EndEffectorMap.IntakeSpeed), "Eject Coral",
                runEndEffectorIntakeCommand(EndEffectorMap.EjectSpeed));
    }

}