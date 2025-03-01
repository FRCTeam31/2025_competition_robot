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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.drivetrain.gyro.IGyro;
import frc.robot.subsystems.climbing.ClimberSubsystem.ClimberMap;

@Logged
public class ClimberIOReal implements IClimberIO {

    private ClimberInputs _inputs = new ClimberInputs();

    private SparkFlex _climbWenchLeftMotor;
    private SparkFlex _climbWenchRightMotor;
    private VictorSPX _climbHooksMotor;
    private DigitalInput _climbOutLimitSwitch;
    private DigitalInput _climbInLimitSwitch;
    private DigitalInput _hooksOutLimitSwitch;
    private DigitalInput _hooksInLimitSwitch;

    public ClimberIOReal() {

        _climbOutLimitSwitch = new DigitalInput(ClimberMap.ClimberOutLimitSwitchChannel);
        _climbInLimitSwitch = new DigitalInput(ClimberMap.ClimberInLimitSwitchChannel);
        _hooksOutLimitSwitch = new DigitalInput(ClimberMap.HooksOutLimitSwitchChannel);
        _hooksInLimitSwitch = new DigitalInput(ClimberMap.HooksInLimitSwitchChannel);

        climbMotorsConfig();
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

    public ClimberInputs updateInputs() {
        double climbWenchMotorSpeed = _climbWenchLeftMotor.get();
        double climbHookMotorSpeed = _climbHooksMotor.getMotorOutputPercent();

        _inputs.ClimbWenchMotorSpeed = climbWenchMotorSpeed;
        _inputs.HooksMotorSpeed = climbHookMotorSpeed;
        _inputs.ClimbWenchOutLimitSwitch = _climbOutLimitSwitch.get();
        _inputs.ClimbWenchInLimitSwitch = _climbInLimitSwitch.get();
        _inputs.HooksClosedLimitSwitch = _hooksOutLimitSwitch.get();
        _inputs.HooksOpenLimitSwitch = _hooksInLimitSwitch.get();

        return _inputs;
    }

    public void setClimbingWenchSpeed(double speed) {
        _climbWenchRightMotor.set(speed);
    }

    public void setHookMotorSpeed(double speed) {
        _climbHooksMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void stopWenchMotors() {
        _climbWenchLeftMotor.stopMotor();
        _climbWenchRightMotor.stopMotor();
    }

    public void stopHooksMotors() {
        _climbHooksMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

}
