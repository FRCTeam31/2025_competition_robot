package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface IAlgaeIntake {

    public void updateInputs(AlgaeIntakeInputsAutoLogged inputs);

    public void setAlgaeMotorSpeed(double speed);

    public void stopMotors();

    public void setAlgaeIntakePosition(Value AlgaeSolenoidPosition);

}
