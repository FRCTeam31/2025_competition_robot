package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem.DrivetrainControlMode;

@Logged
public class DrivetrainOutputs {
  public boolean SnapEnabled = false;
  public Rotation2d SnapSetpoint = new Rotation2d();
  public ChassisSpeeds DesiredChassisSpeeds = new ChassisSpeeds();
  public DrivetrainControlMode ControlMode = DrivetrainControlMode.kRobotRelative;
}
