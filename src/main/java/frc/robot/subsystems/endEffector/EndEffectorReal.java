package frc.robot.subsystems.endEffector;

import org.prime.control.ExtendedPIDConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.drivetrain.SwerveMap;

public class EndEffectorReal implements IEndEffector {

    private SparkFlex _endEffectorIntakeMotor;
    private SparkFlex _endEffectorWristMotor;
    private DigitalInput _coralLimitSwitch;

    public EndEffectorReal() {
        _endEffectorIntakeMotor = new SparkFlex(EndEffectorMap.IntakeMotorCanID, MotorType.kBrushless);
        _endEffectorWristMotor = new SparkFlex(EndEffectorMap.WristMotorCanID, MotorType.kBrushless);
        _coralLimitSwitch = new DigitalInput(EndEffectorMap.LimitSwitchCanID);

    }

    @Override
    public void setIntakeMotorSpeed(double speed) {
        _endEffectorIntakeMotor.set(speed);
    }

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

    public void stopMotors() {
        _endEffectorIntakeMotor.stopMotor();
        _endEffectorWristMotor.stopMotor();
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        var motorIntakeSpeed = _endEffectorIntakeMotor.getAbsoluteEncoder().getVelocity();
        var motorWristSpeed = _endEffectorWristMotor.getAbsoluteEncoder().getVelocity();
        var limitSwitchState = getLimitSwitchState();

        inputs.IntakeMotorSpeed = motorIntakeSpeed;
        inputs.IntakeMotorSpeed = motorWristSpeed;
        inputs.LimitSwitchState = limitSwitchState;
    }

    private boolean getLimitSwitchState() {
        return _coralLimitSwitch.get();
    }
}
