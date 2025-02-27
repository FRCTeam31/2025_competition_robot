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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.drivetrain.SwerveMap;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.climbing.ClimberInputs.ClimberPosition;
import frc.robot.subsystems.climbing.ClimberInputs.ServoPosition;

@Logged
public class ClimberIOReal implements IClimberIO {

    private ClimberInputs m_inputs = new ClimberInputs();

    private SparkFlex _climbLeftMotor;
    private SparkFlex _climbRightMotor;
    private DigitalInput _climbOutLimitSwitch;
    private DigitalInput _climbInLimitSwitch;
    private Servo _climbServo;

    public ClimberIOReal() {

        _climbOutLimitSwitch = new DigitalInput(ClimberMap.ClimberOutLimitSwitchChannel);
        _climbInLimitSwitch = new DigitalInput(ClimberMap.ClimberInLimitSwitchChannel);
        _climbServo = new Servo(ClimberMap.ClimberServoPWMID);
        climbMotorConfig();
    }

    private void climbMotorConfig() {

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        leftMotorConfig.follow(ClimberMap.ClimberRightMotorCANID, true);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        rightMotorConfig.idleMode(IdleMode.kBrake);

        _climbLeftMotor = new SparkFlex(ClimberMap.ClimberLeftMotorCANID, MotorType.kBrushless);
        _climbRightMotor = new SparkFlex(ClimberMap.ClimberRightMotorCANID, MotorType.kBrushless);
        _climbLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _climbRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public ClimberInputs updateInputs() {
        double motorSpeed = _climbLeftMotor.get();

        m_inputs.ClimbMotorSpeed = motorSpeed;
        m_inputs.OutLimitSwitch = _climbOutLimitSwitch.get();
        m_inputs.InLimitSwitch = _climbInLimitSwitch.get();

        return m_inputs;
    }

    public void setMotorSpeed(double speed) {
        _climbRightMotor.set(speed);
    }

    public void stopMotors() {
        _climbLeftMotor.stopMotor();
        _climbRightMotor.stopMotor();
    }

    public void setHooksState(ServoPosition hooksCommmandedPosition) {
        _climbServo.set(hooksCommmandedPosition == ServoPosition.CLOSED ? ClimberMap.ClimberServoClosedValue
                : ClimberMap.ClimberServoOpenValue);
    }

}
