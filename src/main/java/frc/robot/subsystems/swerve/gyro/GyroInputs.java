package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

@AutoLog
public class GyroInputs {
    public Rotation2d Rotation;
    public double AccelerationX;
    public double AccelerationY;
    public double AccelerationZ;
}
