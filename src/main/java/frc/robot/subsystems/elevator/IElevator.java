package frc.robot.subsystems.elevator;

public interface IElevator {
    public void updateInputs(ElevatorInputsAutoLogged inputs);

    public void setMotorVoltages(double volts);

    public void setMotorSpeeds(double output);

    public void resetEncoderPos();

    public void stopMotors();
}
