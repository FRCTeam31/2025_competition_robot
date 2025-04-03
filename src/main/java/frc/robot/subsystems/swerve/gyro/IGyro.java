package frc.robot.subsystems.swerve.gyro;

public interface IGyro {
    public void updateInputs(GyroInputsAutoLogged inputs, double omegaRadiansPerSecond);

    public void reset();

    public void reset(double angle);
}
