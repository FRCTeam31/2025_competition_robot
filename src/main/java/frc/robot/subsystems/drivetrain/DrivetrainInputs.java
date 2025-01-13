package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@Logged
public class DrivetrainInputs {
  public Rotation2d GyroAngle = new Rotation2d();
  public double GyroAccelX = 0;
  public double GyroAccelY = 0;
  public double GyroAccelZ = 0;
  public boolean SnapIsOnTarget = false;
  public ChassisSpeeds RobotRelativeChassisSpeeds = new ChassisSpeeds();
  public Pose2d EstimatedRobotPose = new Pose2d();
  public double SnapCorrectionRadiansPerSecond = 0;
  public SwerveModuleState[] ModuleStates = new SwerveModuleState[4];
}
