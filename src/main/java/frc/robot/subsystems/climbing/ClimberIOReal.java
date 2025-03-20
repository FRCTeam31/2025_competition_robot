package frc.robot.subsystems.climbing;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.drivetrain.gyro.IGyro;
import frc.robot.subsystems.climbing.ClimberSubsystem.ClimberMap;

public class ClimberIOReal implements IClimberIO {

    private SparkFlex _climbWenchLeftMotor;
    private SparkFlex _climbWenchRightMotor;
    private VictorSPX _climbHooksMotor;
    private DigitalInput _climbOutLimitSwitch;
    private DigitalInput _climbInLimitSwitch;
    private DigitalInput _hooksOpenLimitSwitch;
    private DigitalInput _hooksClosedLimitSwitch;

    public ClimberIOReal() {

        _climbOutLimitSwitch = new DigitalInput(ClimberMap.ClimberOutLimitSwitchChannel);
        _climbInLimitSwitch = new DigitalInput(ClimberMap.ClimberInLimitSwitchChannel);
        _hooksOpenLimitSwitch = new DigitalInput(ClimberMap.HooksOpenLimitSwitchChannel);
        _hooksClosedLimitSwitch = new DigitalInput(ClimberMap.HooksClosedLimitSwitchChannel);

        climbMotorsConfig();
        _climbWenchLeftMotor.getEncoder();

    }

    private void climbMotorsConfig() {

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        leftMotorConfig.follow(ClimberMap.ClimberRightMotorCANID, true);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        rightMotorConfig.idleMode(IdleMode.kBrake);

        _climbWenchLeftMotor = new SparkFlex(ClimberMap.ClimberLeftMotorCANID, MotorType.kBrushless);
        _climbWenchRightMotor = new SparkFlex(ClimberMap.ClimberRightMotorCANID, MotorType.kBrushless);
        _climbHooksMotor = new VictorSPX(ClimberMap.ClimberHookMotorCANID);
        _climbWenchLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _climbWenchRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        _climbHooksMotor.configFactoryDefault();

    }

    @Override
    public void updateInputs(ClimberInputsAutoLogged inputs) {
        double climbWenchMotorSpeed = _climbWenchLeftMotor.get();
        double climbHookMotorSpeed = _climbHooksMotor.getMotorOutputPercent();

        inputs.ClimbWenchMotorSpeed = climbWenchMotorSpeed;
        inputs.HooksMotorSpeed = climbHookMotorSpeed;
        inputs.ClimbWenchOutLimitSwitch = _climbOutLimitSwitch.get();
        inputs.ClimbWenchInLimitSwitch = _climbInLimitSwitch.get();
        inputs.HooksClosedLimitSwitch = _hooksClosedLimitSwitch.get();
        inputs.HooksOpenLimitSwitch = !_hooksOpenLimitSwitch.get();
        inputs.climberShaftRotations = getClimberShaftRotations();
    }

    @Override
    public void setClimbingWenchSpeed(double speed) {
        _climbWenchRightMotor.set(speed);
    }

    @Override
    public void setHookMotorSpeed(double speed) {
        _climbHooksMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    @Override
    public void stopWenchMotors() {
        _climbWenchLeftMotor.stopMotor();
        _climbWenchRightMotor.stopMotor();
    }

    @Override
    public void stopHooksMotors() {
        _climbHooksMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    @Override
    public void resetClimberAngle() {
        _climbWenchLeftMotor.getEncoder().setPosition(0);
    }

    public double getClimberShaftRotations() {
        double motorRotations = _climbWenchLeftMotor.getEncoder().getPosition();
        double shaftRotations = motorRotations / 100;
        return shaftRotations;

    }

}
