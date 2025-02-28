package frc.robot.subsystems.drivetrain.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.SwerveMap;

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
        inputs.Pitch = _gyro.getPitch().getValueAsDouble();
    }

    public void reset() {
        var statusCode = _gyro.setYaw(0);
        DriverStation.reportWarning("Reset gyro: " + statusCode.toString(), false);
    }

    public void reset(double angle) {
        var statusCode = _gyro.setYaw(angle);

        DriverStation.reportWarning("Reset gyro to " + angle + ": " + statusCode.toString(), false);
    }

    public double getPitch() {
        return _gyro.getPitch().getValueAsDouble();
    }
}
