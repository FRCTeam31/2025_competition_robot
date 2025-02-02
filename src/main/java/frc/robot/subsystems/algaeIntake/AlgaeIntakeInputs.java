package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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
