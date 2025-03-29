package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class GyroSim implements IGyro {
    private AnalogGyroSim _gyroSim;

    public GyroSim(int channel) {
        _gyroSim = new AnalogGyroSim(channel);
    }

    @Override
    public void updateInputs(GyroInputsAutoLogged inputs, double omegaRadiansPerSecond) {
        // discretize rotational speed
        var delta = omegaRadiansPerSecond * 0.02;
        var currentAngle = Rotation2d.fromDegrees(_gyroSim.getAngle());
        var newGyroAngle = Rotation2d.fromRadians(currentAngle.getRadians() + delta);

        _gyroSim.setAngle(newGyroAngle.getDegrees());

        inputs.Rotation = Rotation2d.fromDegrees(_gyroSim.getAngle());
        inputs.AccelerationX = 0;
        inputs.AccelerationY = 0;
        inputs.AccelerationZ = 0;
    }

    public void reset() {
        _gyroSim.setAngle(0);
    }

    public void reset(double angle) {
        _gyroSim.setAngle(angle);
    }
}
