package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@AutoLog
public class SwerveSubsystemInputs {
  public Rotation2d GyroAngle = new Rotation2d();
  public double GyroAccelX = 0;
  public double GyroAccelY = 0;
  public double GyroAccelZ = 0;
  public ChassisSpeeds RobotRelativeChassisSpeeds = new ChassisSpeeds();
  public Pose2d EstimatedRobotPose = new Pose2d();
  public SwerveModuleState[] ModuleStates = new SwerveModuleState[4];
  public boolean UseAutoAlign;
  public double AutoAlignCorrection = 0;
  public boolean DrivingRobotRelative = false;
}
