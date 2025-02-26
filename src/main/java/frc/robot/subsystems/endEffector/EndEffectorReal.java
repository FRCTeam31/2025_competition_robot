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

    private SparkFlex _endEffectorMotor;
    private DigitalInput _coralLimitSwitch;

    public EndEffectorReal() {
        _endEffectorMotor = new SparkFlex(EndEffectorMap.MotorCanID, MotorType.kBrushless);
        _coralLimitSwitch = new DigitalInput(EndEffectorMap.LimitSwitchCanID);

    }

    @Override
    public void setMotorSpeed(double speed) {
        _endEffectorMotor.set(speed);
    }

    @Override
    public void stopMotors() {
        _endEffectorMotor.stopMotor();
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        var motorSpeed = getMotorSpeed();
        var limitSwitchState = getLimitSwitchState();

        inputs.MotorSpeed = motorSpeed;
        inputs.LimitSwitchState = limitSwitchState;
    }

    private Double getMotorSpeed() {
        return _endEffectorMotor.getAbsoluteEncoder().getVelocity();
    }

    private boolean getLimitSwitchState() {
        return _coralLimitSwitch.get();
    }
}
