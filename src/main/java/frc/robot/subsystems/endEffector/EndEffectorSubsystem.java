package frc.robot.subsystems.endEffector;

import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;

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

    // TODO: How this command is set up now, it will cancel all other running commands in this subsystem when it is ran. This may not be ideal and needs testing. It may be better to always run setWristAngle (in this subsystem's periodic), but update a variable in real/sim (using a command; this.runOnce) that will then make the wrist try to rotate to that new angle. You will not be able to update the variable directly, and will instead need to add a new method to the interface for changing the variable and add it to both real and sim.
    public Command setEndEffectorWristAngleCommand(Rotation2d angle) {
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