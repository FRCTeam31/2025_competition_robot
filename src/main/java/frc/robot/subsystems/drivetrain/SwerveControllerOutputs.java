package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

@AutoLog
public class SwerveControllerOutputs {
  public boolean SnapEnabled = false;
  public Rotation2d SnapSetpoint = new Rotation2d();
  public ChassisSpeeds DesiredChassisSpeeds = new ChassisSpeeds();
}
