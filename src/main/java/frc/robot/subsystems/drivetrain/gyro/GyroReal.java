package frc.robot.subsystems.drivetrain.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.subsystems.drivetrain.DriveMap;

public class GyroReal implements IGyro {
    private Pigeon2 _gyro;

    public GyroReal() {
        _gyro = new Pigeon2(DriveMap.PigeonId);
        var config = new Pigeon2Configuration();
        _gyro.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(GyroInputsAutoLogged inputs, double omegaRadiansPerSecond) {
        inputs.Rotation = _gyro.getRotation3d();
        inputs.AccelerationX = _gyro.getAccelerationX().getValueAsDouble();
        inputs.AccelerationY = _gyro.getAccelerationY().getValueAsDouble();
        inputs.AccelerationZ = _gyro.getAccelerationZ().getValueAsDouble();
    }

    public void reset() {
        _gyro.setYaw(0);
    }

    public void reset(double angle) {
        _gyro.setYaw(angle);
    }
}
