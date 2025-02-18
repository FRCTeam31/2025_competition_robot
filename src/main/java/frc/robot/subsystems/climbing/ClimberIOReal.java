package frc.robot.subsystems.climbing;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.drivetrain.SwerveMap;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@Logged
public class ClimberIOReal implements IClimberIO {

    private ClimberInputs m_inputs = new ClimberInputs();

    private SparkFlex climbLeftMotor;
    private SparkFlex climbRightMotor;
    private DoubleSolenoid climbPneumatics;
    private DigitalInput climbOutLimitSwitch;
    private DigitalInput climbInLimitSwitch;

    public ClimberIOReal() {

        climbPneumatics = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberMap.climberForwardChannel,
                ClimberMap.climberReverseChannel);
        climbOutLimitSwitch = new DigitalInput(ClimberMap.climberOutLimitSwitchChannel);
        climbInLimitSwitch = new DigitalInput(ClimberMap.climberInLimitSwitchChannel);
        climbMotorConfig();
    }

    private void climbMotorConfig() {

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        leftMotorConfig.follow(ClimberMap.climberRightMotorCANID, true);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        rightMotorConfig.idleMode(IdleMode.kBrake);

        climbLeftMotor = new SparkFlex(ClimberMap.climberLeftMotorCANID, MotorType.kBrushless);
        climbRightMotor = new SparkFlex(ClimberMap.climberRightMotorCANID, MotorType.kBrushless);
        climbLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public ClimberInputs updateInputs() {
        Value solenoidState = climbPneumatics.get();
        double motorSpeed = climbLeftMotor.get();

        m_inputs.ClimbMotorSpeed = motorSpeed;
        m_inputs.ClimbSolenoidPosition = solenoidState;
        m_inputs.OutLimitSwitch = climbOutLimitSwitch.get();
        m_inputs.InLimitSwitch = climbInLimitSwitch.get();

        return m_inputs;
    }

    public void setMotorSpeed(double speed) {
        climbRightMotor.set(speed);
    }

    public void stopMotors() {
        climbLeftMotor.stopMotor();
        climbRightMotor.stopMotor();
    }

    public void setClimbersState(Value climberState) {
        climbPneumatics.set(climberState);
    }
}
