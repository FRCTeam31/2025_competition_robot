package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@Logged
public class AlgaeIntakeInputs {

    public double AlgaeMotorSpeed = 0;
    public Value AlgaeLeftSolenoidPosition;
    public Value AlgaeRightSolenoidPosition;

    public AlgaeIntakeInputs() {
    }

    public AlgaeIntakeInputs(double algaeMotorSpeed, Value algaeLeftSolenoidPosition,
            Value algaeRightSolenoidPosition) {
        AlgaeMotorSpeed = algaeMotorSpeed;
        AlgaeLeftSolenoidPosition = algaeLeftSolenoidPosition;
        AlgaeRightSolenoidPosition = algaeRightSolenoidPosition;
    }
}
