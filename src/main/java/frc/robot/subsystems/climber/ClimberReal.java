package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberReal implements IClimber {

    private SparkFlex _climbWinchLeftMotor;
    private SparkFlex _climbWinchRightMotor;
    private VictorSPX _climbHooksMotor;

    private DigitalInput _climbOutLimitSwitch;
    private DigitalInput _climbInLimitSwitch;
    private DigitalInput _hooksOpenLimitSwitch;
    private DigitalInput _hooksClosedLimitSwitch;

    public ClimberReal() {
        _climbOutLimitSwitch = new DigitalInput(ClimberMap.ClimberOutLimitSwitchChannel);
        _climbInLimitSwitch = new DigitalInput(ClimberMap.ClimberInLimitSwitchChannel);
        _hooksOpenLimitSwitch = new DigitalInput(ClimberMap.HooksOpenLimitSwitchChannel);
        _hooksClosedLimitSwitch = new DigitalInput(ClimberMap.HooksClosedLimitSwitchChannel);

        climbMotorsConfig();
    }

    private void climbMotorsConfig() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.follow(ClimberMap.ClimberRightMotorCANID, true);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        // leftMotorConfig.encoder.positionConversionFactor(1 / ClimberMap.ClimberGearRatio);
        _climbWinchLeftMotor = new SparkFlex(ClimberMap.ClimberLeftMotorCANID, MotorType.kBrushless);
        _climbWinchLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.idleMode(IdleMode.kBrake);
        _climbWinchRightMotor = new SparkFlex(ClimberMap.ClimberRightMotorCANID, MotorType.kBrushless);
        _climbWinchRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        _climbHooksMotor = new VictorSPX(ClimberMap.ClimberHookMotorCANID);
        _climbHooksMotor.configFactoryDefault();
    }

    @Override
    public void updateInputs(ClimberInputsAutoLogged inputs) {
        inputs.WinchMotorSpeed = _climbWinchLeftMotor.get();
        inputs.HooksMotorSpeed = _climbHooksMotor.getMotorOutputPercent();
        inputs.WinchOuterLimitSwitch = _climbOutLimitSwitch.get();
        inputs.WinchInnerLimitSwitch = _climbInLimitSwitch.get();
        inputs.HooksClosedLimitSwitch = _hooksClosedLimitSwitch.get();
        inputs.HooksOpenLimitSwitch = !_hooksOpenLimitSwitch.get();
        inputs.climberShaftRotations = getClimberShaftRotations();
    }

    @Override
    public void setWinchSpeed(double speed) {
        _climbWinchRightMotor.set(speed);
    }

    @Override
    public void setHookMotorSpeed(double speed) {
        _climbHooksMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    @Override
    public void stopWinchMotors() {
        _climbWinchLeftMotor.stopMotor();
        _climbWinchRightMotor.stopMotor();
    }

    @Override
    public void stopHooksMotors() {
        _climbHooksMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    @Override
    public void resetClimberAngle() {
        _climbWinchLeftMotor.getEncoder().setPosition(0);
    }

    public double getClimberShaftRotations() {
        // Scaled to gear ratio in config
        return _climbWinchLeftMotor.getEncoder().getPosition() / ClimberMap.ClimberGearRatio;
    }
}
