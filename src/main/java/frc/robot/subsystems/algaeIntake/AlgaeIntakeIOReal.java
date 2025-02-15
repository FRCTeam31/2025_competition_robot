package frc.robot.subsystems.algaeIntake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeMap;

@Logged
public class AlgaeIntakeIOReal implements IAlgaeIntakeIO {

    private AlgaeIntakeInputs m_inputs = new AlgaeIntakeInputs();

    public SparkFlex algaeIntakeMotor;
    public DoubleSolenoid leftAlgaeSolenoid;
    public DoubleSolenoid rightAlgaeSolenoid;

    public AlgaeIntakeIOReal() {
        algaeIntakeMotor = new SparkFlex(AlgaeIntakeMap.algaeIntakeMotorCANID, MotorType.kBrushless);
        leftAlgaeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                AlgaeIntakeMap.leftAlgaeSolenoidForwardChannel, AlgaeIntakeMap.leftAlgaeSolenoidReverseChannel);
        rightAlgaeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                AlgaeIntakeMap.rightAlgaeSolenoidForwardChannel, AlgaeIntakeMap.rightAlgaeSolenoidReverseChannel);
    }

    public AlgaeIntakeInputs getInputs() {
        Value leftSolenoidPosition = leftAlgaeSolenoid.get();
        Value rightSolenoidPosition = rightAlgaeSolenoid.get();
        double motorSpeed = algaeIntakeMotor.get();

        m_inputs.AlgaeMotorSpeed = motorSpeed;
        m_inputs.AlgaeLeftSolenoidPosition = leftSolenoidPosition;
        m_inputs.AlgaeRightSolenoidPosition = rightSolenoidPosition;

        return m_inputs;
    }

    public void setAlgaeMotorSpeed(double speed) {
        algaeIntakeMotor.set(speed);
    }

    public void stopMotors() {
        algaeIntakeMotor.stopMotor();
    }

    public void setAlgaeIntakePosition(Value algaeSolenoidPosition) {
        leftAlgaeSolenoid.set(algaeSolenoidPosition);
        rightAlgaeSolenoid.set(algaeSolenoidPosition);
    }
}
