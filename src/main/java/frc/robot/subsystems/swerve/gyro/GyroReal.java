package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveMap;

public class GyroReal implements IGyro {
    private Pigeon2 _gyro;

    public GyroReal() {
        _gyro = new Pigeon2(SwerveMap.PigeonId);
        var config = new Pigeon2Configuration();
        _gyro.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(GyroInputsAutoLogged inputs, double omegaRadiansPerSecond) {
        inputs.Rotation = _gyro.getRotation2d();
        inputs.AccelerationX = _gyro.getAccelerationX().getValueAsDouble();
        inputs.AccelerationY = _gyro.getAccelerationY().getValueAsDouble();
        inputs.AccelerationZ = _gyro.getAccelerationZ().getValueAsDouble();
    }

    public void reset() {
        var statusCode = _gyro.setYaw(0);
        DriverStation.reportWarning("Reset gyro: " + statusCode.toString(), false);
    }

    public void reset(double angle) {
        var statusCode = _gyro.setYaw(angle);

        DriverStation.reportWarning("Reset gyro to " + angle + ": " + statusCode.toString(), false);
    }
}
