package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@Logged
public interface IAlgaeIntakeIO {

    public AlgaeIntakeInputs getInputs();

    public void setAlgaeMotorSpeed(double speed);

    public void stopMotors();

    public void setAlgaeIntakePosition(Value AlgaeSolenoidPosition);

}
