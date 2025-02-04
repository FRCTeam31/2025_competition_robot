package frc.robot.subsystems.elevator;

import frc.robot.Enums.ElevatorLocation;

public interface IElevatorIO {

    public ElevatorInputs getInputs();

    public void SetElevatorPosition(ElevatorLocation location);

    public void StopMotors();

}
