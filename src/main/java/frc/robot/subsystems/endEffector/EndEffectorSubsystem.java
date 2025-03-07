package frc.robot.subsystems.endEffector;

import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;

import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

        public static final double GearRatio = 5;

        public static final double MaxWristAngle = 135;
        public static final double MinWristAngle = 0;

    }

    private IEndEffector _endEffector;

    private PIDController _wristPidController = new PIDController(EndEffectorMap.WristPID.kP,
            EndEffectorMap.WristPID.kI,
            EndEffectorMap.WristPID.kD);

    private EndEffectorInputsAutoLogged _inputs = new EndEffectorInputsAutoLogged();

    private void setWristSetpoint(double setpoint) {
        _wristPidController
                .setSetpoint(MathUtil.clamp(setpoint, EndEffectorMap.MinWristAngle, EndEffectorMap.MaxWristAngle));
    }

    public EndEffectorSubsystem(boolean isReal) {
        setName("End Effector");
        _endEffector = isReal ? new EndEffectorReal() : new EndEffectorSim();
    }

    public void seekWristAngle() {
        double PIDOutput = _wristPidController.calculate(_inputs.EndEffectorAngleDegrees);
        PIDOutput = MathUtil.clamp(PIDOutput, -1, 1);
        _endEffector.setWristMotorSpeed(PIDOutput);
    }

    @Override
    public void periodic() {
        _endEffector.updateInputs(_inputs);
        Logger.processInputs(getName(), _inputs);
    }

    public Command runEndEffectorIntakeCommand(double speed) {
        return this.run(() -> {
            _endEffector.setIntakeMotorSpeed(speed);
        });
    }

    public Command stopEndEffectorIntakeCommand() {
        return this.run(() -> {
            _endEffector.stopIntakeMotor();
        });
    }

    public Map<String, Command> getNamedCommands() {
        return Map.of("Stop Intake", stopEndEffectorIntakeCommand(), "Intake Coral",
                runEndEffectorIntakeCommand(EndEffectorMap.IntakeSpeed), "Eject Coral",
                runEndEffectorIntakeCommand(EndEffectorMap.EjectSpeed));
    }

}