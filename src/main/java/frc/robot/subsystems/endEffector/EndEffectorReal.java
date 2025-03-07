package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem.EndEffectorMap;

public class EndEffectorReal implements IEndEffector {

    private SparkFlex _endEffectorIntakeMotor;
    private SparkFlex _endEffectorWristMotor;
    private DigitalInput _coralLimitSwitch;

    public EndEffectorReal() {
        _coralLimitSwitch = new DigitalInput(EndEffectorMap.LimitSwitchCanID);

    }

    public void configureEndEffectorMotors() {
        _endEffectorIntakeMotor = new SparkFlex(EndEffectorMap.IntakeMotorCanID, MotorType.kBrushless);
        _endEffectorWristMotor = new SparkFlex(EndEffectorMap.WristMotorCanID, MotorType.kBrushless);
        SparkFlexConfig wristConfig = new SparkFlexConfig();
        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        wristConfig.smartCurrentLimit(30);
        intakeConfig.smartCurrentLimit(20);
        wristConfig.idleMode(IdleMode.kBrake);
        intakeConfig.idleMode(IdleMode.kBrake);
        _endEffectorIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _endEffectorWristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void setIntakeMotorSpeed(double speed) {
        _endEffectorIntakeMotor.set(speed);
    }

    @Override
    public void setWristMotorSpeed(double speed) {
        _endEffectorWristMotor.set(speed);
    }

    @Override
    public void stopIntakeMotor() {
        _endEffectorIntakeMotor.stopMotor();
    }

    @Override
    public void stopWristMotor() {
        _endEffectorWristMotor.stopMotor();
    }

    @Override
    public void stopMotors() {
        _endEffectorIntakeMotor.stopMotor();
        _endEffectorWristMotor.stopMotor();
    }

    private double getWristAngleDegrees() {

        double wristPosition = _endEffectorWristMotor.getEncoder().getPosition() / EndEffectorMap.GearRatio;

        return Rotation2d.fromRotations(wristPosition).getDegrees();
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        var motorIntakeSpeed = _endEffectorIntakeMotor.getAbsoluteEncoder().getVelocity();
        var motorWristSpeed = _endEffectorWristMotor.getAbsoluteEncoder().getVelocity();
        var limitSwitchState = getLimitSwitchState();

        inputs.IntakeMotorSpeed = motorIntakeSpeed;
        inputs.IntakeMotorSpeed = motorWristSpeed;
        inputs.LimitSwitchState = limitSwitchState;
        inputs.EndEffectorAngleDegrees = getWristAngleDegrees();
    }

    private boolean getLimitSwitchState() {
        return _coralLimitSwitch.get();
    }
}
