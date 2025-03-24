package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.gyro.GyroReal;
import frc.robot.subsystems.drivetrain.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.drivetrain.gyro.GyroSim;
import frc.robot.subsystems.drivetrain.gyro.IGyro;
import frc.robot.subsystems.drivetrain.swervemodule.ISwerveModule;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleReal;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleSim;

/**
 * A class that controls the swerve modules and gyro
 */
public class SwerveIOPackager {

  public SwerveDriveKinematics Kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;

  private IGyro _gyro;
  private double _simGyroOmega = 0;
  private ISwerveModule _frontLeftModule, _frontRightModule, _rearLeftModule, _rearRightModule;

  private GyroInputsAutoLogged _gyroInputs = new GyroInputsAutoLogged();
  private SwerveModuleInputsAutoLogged[] m_moduleInputs = new SwerveModuleInputsAutoLogged[] {
      new SwerveModuleInputsAutoLogged(),
      new SwerveModuleInputsAutoLogged(),
      new SwerveModuleInputsAutoLogged(),
      new SwerveModuleInputsAutoLogged()
  };

  public SwerveIOPackager(boolean isReal) {
    // Create kinematics in order FL, FR, RL, RR
    Kinematics = new SwerveDriveKinematics(SwerveMap.FrontLeftSwerveModule.ModuleLocation,
        SwerveMap.FrontRightSwerveModule.ModuleLocation,
        SwerveMap.RearLeftSwerveModule.ModuleLocation,
        SwerveMap.RearRightSwerveModule.ModuleLocation);

    _gyro = isReal
        ? new GyroReal()
        : new GyroSim(0);
    _gyro.updateInputs(_gyroInputs, 0);

    _frontLeftModule = isReal
        ? new SwerveModuleReal("FrontLeftModule", SwerveMap.FrontLeftSwerveModule)
        : new SwerveModuleSim("FrontLeftModule", SwerveMap.FrontLeftSwerveModule);
    _frontRightModule = isReal
        ? new SwerveModuleReal("FrontRightModule", SwerveMap.FrontRightSwerveModule)
        : new SwerveModuleSim("FrontRightModule", SwerveMap.FrontRightSwerveModule);
    _rearLeftModule = isReal
        ? new SwerveModuleReal("RearLeftModule", SwerveMap.RearLeftSwerveModule)
        : new SwerveModuleSim("RearLeftModule", SwerveMap.RearLeftSwerveModule);
    _rearRightModule = isReal
        ? new SwerveModuleReal("RearRightModule", SwerveMap.RearRightSwerveModule)
        : new SwerveModuleSim("RearRightModule", SwerveMap.RearRightSwerveModule);

    // Create pose estimator
    m_poseEstimator = new SwerveDrivePoseEstimator(Kinematics, _gyroInputs.Rotation,
        getModulePositions(), new Pose2d());

    // Store current Drive and Steering PID values in Preferences
    Preferences.initDouble("DriveKp", SwerveMap.DrivePID.kP);
    Preferences.initDouble("DriveKi", SwerveMap.DrivePID.kI);
    Preferences.initDouble("DriveKd", SwerveMap.DrivePID.kD);
    Preferences.initDouble("DriveKs", SwerveMap.DrivePID.kS);
    Preferences.initDouble("DriveKv", SwerveMap.DrivePID.kV);
    Preferences.initDouble("DriveKa", SwerveMap.DrivePID.kA);
    Preferences.initDouble("SteerKp", SwerveMap.SteeringPID.kP);
    Preferences.initDouble("SteerKi", SwerveMap.SteeringPID.kI);
    Preferences.initDouble("SteerKd", SwerveMap.SteeringPID.kD);
  }

  /**
   * Called periodically to update the swerve modules and gyro
   * @param swerveInputs
   */
  public void updateInputs(SwerveSubsystemInputsAutoLogged swerveInputs) {
    _frontLeftModule.updateInputs(m_moduleInputs[0]);
    Logger.processInputs("Drivetrain/FLModule", m_moduleInputs[0]);
    _frontRightModule.updateInputs(m_moduleInputs[1]);
    Logger.processInputs("Drivetrain/FRModule", m_moduleInputs[1]);
    _rearLeftModule.updateInputs(m_moduleInputs[2]);
    Logger.processInputs("Drivetrain/RLModule", m_moduleInputs[2]);
    _rearRightModule.updateInputs(m_moduleInputs[3]);
    Logger.processInputs("Drivetrain/RRModule", m_moduleInputs[3]);

    swerveInputs.ModuleStates = getModuleStates();
    swerveInputs.RobotRelativeChassisSpeeds = Kinematics.toChassisSpeeds(getModuleStates());

    _gyro.updateInputs(_gyroInputs, _simGyroOmega);
    swerveInputs.GyroAngle = _gyroInputs.Rotation;
    swerveInputs.GyroAccelX = _gyroInputs.AccelerationX;
    swerveInputs.GyroAccelY = _gyroInputs.AccelerationY;
    swerveInputs.GyroAccelZ = _gyroInputs.AccelerationZ;

    var modulePositions = getModulePositions();
    swerveInputs.EstimatedRobotPose = m_poseEstimator.update(swerveInputs.GyroAngle, modulePositions);

    checkPreferences();
  }

  /**
   * Checks the preferences for any changes and updates the PID values in each swerve module if necessary
   */
  private void checkPreferences() {
    var driveKpChanged = Preferences.getDouble("DriveKp", SwerveMap.DrivePID.kP) != SwerveMap.DrivePID.kP;
    var driveKiChanged = Preferences.getDouble("DriveKi", SwerveMap.DrivePID.kI) != SwerveMap.DrivePID.kI;
    var driveKdChanged = Preferences.getDouble("DriveKd", SwerveMap.DrivePID.kD) != SwerveMap.DrivePID.kD;
    var driveKsChanged = Preferences.getDouble("DriveKs", SwerveMap.DrivePID.kS) != SwerveMap.DrivePID.kS;
    var driveKvChanged = Preferences.getDouble("DriveKv", SwerveMap.DrivePID.kV) != SwerveMap.DrivePID.kV;
    var driveKaChanged = Preferences.getDouble("DriveKa", SwerveMap.DrivePID.kA) != SwerveMap.DrivePID.kA;

    if (driveKpChanged || driveKiChanged || driveKdChanged || driveKsChanged || driveKvChanged || driveKaChanged) {
      SwerveMap.DrivePID.kP = Preferences.getDouble("DriveKp", SwerveMap.DrivePID.kP);
      SwerveMap.DrivePID.kI = Preferences.getDouble("DriveKi", SwerveMap.DrivePID.kI);
      SwerveMap.DrivePID.kD = Preferences.getDouble("DriveKd", SwerveMap.DrivePID.kD);
      SwerveMap.DrivePID.kS = Preferences.getDouble("DriveKs", SwerveMap.DrivePID.kS);
      SwerveMap.DrivePID.kV = Preferences.getDouble("DriveKv", SwerveMap.DrivePID.kV);
      SwerveMap.DrivePID.kA = Preferences.getDouble("DriveKa", SwerveMap.DrivePID.kA);

      _frontLeftModule.setDrivePID(SwerveMap.DrivePID);
      _frontRightModule.setDrivePID(SwerveMap.DrivePID);
      _rearLeftModule.setDrivePID(SwerveMap.DrivePID);
      _rearRightModule.setDrivePID(SwerveMap.DrivePID);
    }

    var steerKpChanged = Preferences.getDouble("SteerKp", SwerveMap.SteeringPID.kP) != SwerveMap.SteeringPID.kP;
    var steerKiChanged = Preferences.getDouble("SteerKi", SwerveMap.SteeringPID.kI) != SwerveMap.SteeringPID.kI;
    var steerKdChanged = Preferences.getDouble("SteerKd", SwerveMap.SteeringPID.kD) != SwerveMap.SteeringPID.kD;
    if (steerKpChanged || steerKiChanged || steerKdChanged) {
      SwerveMap.SteeringPID.kP = Preferences.getDouble("SteerKp", SwerveMap.SteeringPID.kP);
      SwerveMap.SteeringPID.kI = Preferences.getDouble("SteerKi", SwerveMap.SteeringPID.kI);
      SwerveMap.SteeringPID.kD = Preferences.getDouble("SteerKd", SwerveMap.SteeringPID.kD);

      _frontLeftModule.setSteeringPID(SwerveMap.SteeringPID);
      _frontRightModule.setSteeringPID(SwerveMap.SteeringPID);
      _rearLeftModule.setSteeringPID(SwerveMap.SteeringPID);
      _rearRightModule.setSteeringPID(SwerveMap.SteeringPID);
    }
  }

  /**
   * Sets the desired states for each swerve module in order FL, FR, RL, RR
   * 
   * @param desiredStates
   */
  public void setDesiredModuleStates(SwerveModuleState[] desiredStates) {
    _frontLeftModule.setDesiredState(desiredStates[0]);
    _frontRightModule.setDesiredState(desiredStates[1]);
    _rearLeftModule.setDesiredState(desiredStates[2]);
    _rearRightModule.setDesiredState(desiredStates[3]);
  }

  /**
   * Sets the omega of the gyro sim for simulation purposes
   * 
   * @param omega
   */
  public void setSimGyroOmega(double omega) {
    if (_gyro instanceof GyroSim) {
      _simGyroOmega = omega;
    }
  }

  /**
   * Stops all motors on the swerve modules
   */
  public void stopAllMotors() {
    _frontLeftModule.stopMotors();
    _frontRightModule.stopMotors();
    _rearLeftModule.stopMotors();
    _rearRightModule.stopMotors();
  }

  /**
   * Resets the gyro and pose estimator
   */
  public void resetGyro() {
    _gyro.reset(Robot.onBlueAlliance() ? 180 : 0);
    _gyro.updateInputs(_gyroInputs, 0);
    m_poseEstimator.resetPosition(_gyroInputs.Rotation, getModulePositions(),
        m_poseEstimator.getEstimatedPosition());
  }

  /**
   * Sets the pose estimator's pose
   * @param pose
   */
  public void setEstimatorPose(Pose2d pose) {
    m_poseEstimator.resetPosition(_gyroInputs.Rotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator
   * @param pose The estimated pose from vision
   * @param timestamp The timestamp of the vision measurement
   * @param stdDeviations The standard deviations of the vision measurement
   */
  public void addPoseEstimatorVisionMeasurement(Pose2d pose, double timestamp,
      Matrix<N3, N1> stdDeviations) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp, stdDeviations);
  }

  /**
   * Gets the measured states for each swerve module in FL, FR, RL, RR order
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_moduleInputs[0].ModuleState,
        m_moduleInputs[1].ModuleState,
        m_moduleInputs[2].ModuleState,
        m_moduleInputs[3].ModuleState,
    };
  }

  /**
   * Gets the measured positions for each swerve module in FL, FR, RL, RR order
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_moduleInputs[0].ModulePosition,
        m_moduleInputs[1].ModulePosition,
        m_moduleInputs[2].ModulePosition,
        m_moduleInputs[3].ModulePosition,
    };
  }
}
