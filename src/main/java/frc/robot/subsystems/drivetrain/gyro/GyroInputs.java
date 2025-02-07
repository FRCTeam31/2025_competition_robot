package frc.robot.subsystems.drivetrain.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation3d;

@AutoLog
public class GyroInputs {
    public Rotation3d Rotation;
    public double AccelerationX;
    public double AccelerationY;
    public double AccelerationZ;
}
