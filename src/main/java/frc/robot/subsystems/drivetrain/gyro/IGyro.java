package frc.robot.subsystems.drivetrain.gyro;

public interface IGyro {
    public void updateInputs(GyroInputsAutoLogged inputs, double omegaRadiansPerSecond);

    public void reset();

    public void reset(double angle);
}
