package frc.robot.subsystems.climbing;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.maps.ClimberMap;

public class ClimberIOReal implements IClimberIO {

    public ClimberInputs m_inputs = new ClimberInputs();

    public SparkFlex climbMotor;
    public DoubleSolenoid climbPneumatics;

    public ClimberIOReal() {

        climbMotor = new SparkFlex(0, MotorType.kBrushless);
        climbPneumatics = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberMap.climberForwardChannel,
                ClimberMap.climberReverseChannel);

    }

    public ClimberInputs getInputs() {
        Value solenoidState = climbPneumatics.get();
        double motorSpeed = climbMotor.get();

        m_inputs.ClimbMotorSpeed = motorSpeed;
        m_inputs.ClimbSolenoidPosition = solenoidState;

        return m_inputs;
    }

    public void setMotorSpeed(double speed) {
        climbMotor.set(speed);
    }

    public void stopMotors() {
        climbMotor.stopMotor();
    }

    public void setClimbersState(Value climberState) {
        climbPneumatics.set(climberState);
    }
}
