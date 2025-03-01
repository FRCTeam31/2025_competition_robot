package frc.robot.subsystems.algaeIntake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem.AlgaeIntakeMap;

public class AlgaeIntakeReal implements IAlgaeIntake {

    public SparkFlex _algaeIntakeMotor;
    public DoubleSolenoid _leftAlgaeSolenoid;
    public DoubleSolenoid _rightAlgaeSolenoid;

    public AlgaeIntakeReal() {
        _algaeIntakeMotor = new SparkFlex(AlgaeIntakeMap.algaeIntakeMotorCANID, MotorType.kBrushless);
        _leftAlgaeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                AlgaeIntakeMap.leftAlgaeSolenoidForwardChannel, AlgaeIntakeMap.leftAlgaeSolenoidReverseChannel);
        _rightAlgaeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                AlgaeIntakeMap.rightAlgaeSolenoidForwardChannel, AlgaeIntakeMap.rightAlgaeSolenoidReverseChannel);
    }

    public void updateInputs(AlgaeIntakeInputsAutoLogged inputs) {
        inputs.AlgaeLeftSolenoidPosition = _leftAlgaeSolenoid.get();
        inputs.AlgaeRightSolenoidPosition = _rightAlgaeSolenoid.get();
        inputs.AlgaeMotorSpeed = _algaeIntakeMotor.get();
    }

    public void setAlgaeMotorSpeed(double speed) {
        _algaeIntakeMotor.set(speed);
    }

    public void stopMotors() {
        _algaeIntakeMotor.stopMotor();
    }

    public void setAlgaeIntakePosition(Value algaeSolenoidPosition) {
        _leftAlgaeSolenoid.set(algaeSolenoidPosition);
        _rightAlgaeSolenoid.set(algaeSolenoidPosition);
    }
}
