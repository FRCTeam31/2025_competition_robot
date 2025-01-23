package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Robot;
import frc.robot.maps.DriveMap;
import frc.robot.subsystems.drivetrain.gyro.GyroReal;
import frc.robot.subsystems.drivetrain.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.drivetrain.gyro.GyroSim;
import frc.robot.subsystems.drivetrain.gyro.IGyro;
import frc.robot.subsystems.drivetrain.swervemodule.ISwerveModule;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleOutputsAutoLogged;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleReal;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleSim;

public class SwerveController {

  public SwerveDriveKinematics Kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;

  private IGyro _gyro;
  private ISwerveModule m_frontLeftModule, m_frontRightModule, m_rearLeftModule,
      m_rearRightModule;

  private GyroInputsAutoLogged _gyroInputs = new GyroInputsAutoLogged();
  private SwerveModuleInputsAutoLogged[] m_moduleInputs = new SwerveModuleInputsAutoLogged[] {
      new SwerveModuleInputsAutoLogged(),
      new SwerveModuleInputsAutoLogged(),
      new SwerveModuleInputsAutoLogged(),
      new SwerveModuleInputsAutoLogged()
  };
  private SwerveModuleOutputsAutoLogged[] m_moduleOutputs = new SwerveModuleOutputsAutoLogged[] {
      new SwerveModuleOutputsAutoLogged(),
      new SwerveModuleOutputsAutoLogged(),
      new SwerveModuleOutputsAutoLogged(),
      new SwerveModuleOutputsAutoLogged() };

  public SwerveController(boolean isReal) {
    // Create kinematics in order FL, FR, RL, RR
    Kinematics = new SwerveDriveKinematics(DriveMap.FrontLeftSwerveModule.ModuleLocation,
        DriveMap.FrontRightSwerveModule.ModuleLocation,
        DriveMap.RearLeftSwerveModule.ModuleLocation,
        DriveMap.RearRightSwerveModule.ModuleLocation);

    _gyro = isReal
        ? new GyroReal()
        : new GyroSim(0);
    _gyro.updateInputs(_gyroInputs, 0);

    m_frontLeftModule = isReal
        ? new SwerveModuleReal(DriveMap.FrontLeftSwerveModule)
        : new SwerveModuleSim(DriveMap.FrontLeftSwerveModule);
    m_frontRightModule = isReal
        ? new SwerveModuleReal(DriveMap.FrontRightSwerveModule)
        : new SwerveModuleSim(DriveMap.FrontRightSwerveModule);
    m_rearLeftModule = isReal
        ? new SwerveModuleReal(DriveMap.RearLeftSwerveModule)
        : new SwerveModuleSim(DriveMap.RearLeftSwerveModule);
    m_rearRightModule = isReal
        ? new SwerveModuleReal(DriveMap.RearRightSwerveModule)
        : new SwerveModuleSim(DriveMap.RearRightSwerveModule);

    // Create pose estimator
    m_poseEstimator = new SwerveDrivePoseEstimator(Kinematics, _gyroInputs.Rotation.toRotation2d(),
        getModulePositions(), new Pose2d());
  }

  public void updateInputs(SwerveControllerInputsAutoLogged inputs) {
    m_frontLeftModule.updateInputs(m_moduleInputs[0]);
    m_frontRightModule.updateInputs(m_moduleInputs[1]);
    m_rearLeftModule.updateInputs(m_moduleInputs[2]);
    m_rearRightModule.updateInputs(m_moduleInputs[3]);
    inputs.ModuleStates = getModuleStates();

    inputs.RobotRelativeChassisSpeeds = Kinematics.toChassisSpeeds(getModuleStates());
    inputs.GyroAngle = _gyroInputs.Rotation.toRotation2d();
    inputs.GyroAccelX = _gyroInputs.AccelerationX;
    inputs.GyroAccelY = _gyroInputs.AccelerationY;
    inputs.GyroAccelZ = _gyroInputs.AccelerationZ;

    var modulePositions = getModulePositions();
    inputs.EstimatedRobotPose = m_poseEstimator.update(inputs.GyroAngle, modulePositions);
  }

  public void setDriveVoltages(Voltage volts) {
    var angle = Rotation2d.fromDegrees(0);
    m_frontLeftModule.setDriveVoltage(volts.magnitude(), angle);
    m_frontRightModule.setDriveVoltage(volts.magnitude(), angle);
    m_rearLeftModule.setDriveVoltage(volts.magnitude(), angle);
    m_rearRightModule.setDriveVoltage(volts.magnitude(), angle);
  }

  /**
   * Sets the desired states for each swerve module in order FL, FR, RL, RR
   * 
   * @param desiredStates
   */
  public void setDesiredModuleStates(SwerveModuleState[] desiredStates) {
    m_moduleOutputs[0].DesiredState = desiredStates[0];
    m_moduleOutputs[1].DesiredState = desiredStates[1];
    m_moduleOutputs[2].DesiredState = desiredStates[2];
    m_moduleOutputs[3].DesiredState = desiredStates[3];
  }

  public void stopAllMotors() {
    m_frontLeftModule.stopMotors();
    m_frontRightModule.stopMotors();
    m_rearLeftModule.stopMotors();
    m_rearRightModule.stopMotors();
  }

  public void resetGyro() {
    _gyro.reset(Robot.onBlueAlliance() ? 180 : 0);
    _gyro.updateInputs(_gyroInputs, 0);
    m_poseEstimator.resetPosition(_gyroInputs.Rotation.toRotation2d(), getModulePositions(),
        m_poseEstimator.getEstimatedPosition());
  }

  public void setEstimatorPose(Pose2d pose) {
    m_poseEstimator.resetPosition(_gyroInputs.Rotation.toRotation2d(), getModulePositions(), pose);
  }

  public void addPoseEstimatorVisionMeasurement(Pose2d pose, double timestamp,
      Matrix<N3, N1> stdDeviations) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp, stdDeviations);
  }

  public void logSysIdDrive(SysIdRoutineLog log) {
    // Record a frame for the left motors. Since these share an encoder, we consider
    // the entire group to be one motor.
    log.motor("Front-Left-Module").voltage(Units.Volts.of(m_moduleInputs[0].DriveMotorVoltage)) // measured motor voltage
        .linearPosition(Units.Meters.of(m_moduleInputs[0].ModulePosition.distanceMeters)) // distance in meters
        .linearVelocity(
            Units.MetersPerSecond.of(m_moduleInputs[0].ModuleState.speedMetersPerSecond)); // speed in meters per second

  }

  /**
   * Gets the instantaneous states for each swerve module in FL, FR, RL, RR order
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] { m_moduleInputs[0].ModuleState, m_moduleInputs[1].ModuleState,
        m_moduleInputs[2].ModuleState, m_moduleInputs[3].ModuleState, };
  }

  /**
   * Gets the cumulative positions for each swerve module in FL, FR, RL, RR order
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { m_moduleInputs[0].ModulePosition,
        m_moduleInputs[1].ModulePosition, m_moduleInputs[2].ModulePosition,
        m_moduleInputs[3].ModulePosition, };
  }
}
