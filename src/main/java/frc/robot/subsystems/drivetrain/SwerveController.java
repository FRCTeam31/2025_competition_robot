package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Robot;
import frc.robot.maps.DriveMap;
import frc.robot.subsystems.drivetrain.swervemodule.ISwerveModuleIO;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleIOInputs;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleIOOutputs;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleIOReal;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleIOSim;

@Logged(strategy = Strategy.OPT_IN)
public class SwerveController {

  private boolean _robotIsReal = false;
  private DrivetrainInputs _inputs;
  private DrivetrainOutputs _outputs;

  private Pigeon2 m_gyro;
  private AnalogGyroSim m_gyroSim;

  @Logged(name = "SnapAnglePIDController", importance = Logged.Importance.CRITICAL)
  private PIDController m_snapAngleController;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;

  private ISwerveModuleIO m_frontLeftModule, m_frontRightModule, m_rearLeftModule,
      m_rearRightModule;

  @Logged(name = "ModuleInputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOInputs[] m_moduleInputs = new SwerveModuleIOInputs[] { new SwerveModuleIOInputs(),
      new SwerveModuleIOInputs(),
      new SwerveModuleIOInputs(), new SwerveModuleIOInputs() };

  @Logged(name = "ModuleOutputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOOutputs[] m_moduleOutputs = new SwerveModuleIOOutputs[] { new SwerveModuleIOOutputs(),
      new SwerveModuleIOOutputs(),
      new SwerveModuleIOOutputs(), new SwerveModuleIOOutputs() };

  public SwerveController(boolean isReal) {
    _robotIsReal = isReal;
    _inputs = new DrivetrainInputs();

    // Create kinematics in order FL, FR, RL, RR
    m_kinematics = new SwerveDriveKinematics(DriveMap.FrontLeftSwerveModule.ModuleLocation,
        DriveMap.FrontRightSwerveModule.ModuleLocation,
        DriveMap.RearLeftSwerveModule.ModuleLocation,
        DriveMap.RearRightSwerveModule.ModuleLocation);

    if (_robotIsReal) {
      // Create gyro
      m_gyro = new Pigeon2(DriveMap.PigeonId);
      m_gyro.getConfigurator().apply(new Pigeon2Configuration());

      // Create swerve modules in CCW order from FL to FR
      m_frontLeftModule = new SwerveModuleIOReal(DriveMap.FrontLeftSwerveModule);
      m_frontRightModule = new SwerveModuleIOReal(DriveMap.FrontRightSwerveModule);
      m_rearLeftModule = new SwerveModuleIOReal(DriveMap.RearLeftSwerveModule);
      m_rearRightModule = new SwerveModuleIOReal(DriveMap.RearRightSwerveModule);

      // Create pose estimator
      m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(),
          getModulePositions(), new Pose2d());
    } else {
      m_gyroSim = new AnalogGyroSim(new AnalogGyro(0));

      // Create swerve modules in CCW order from FL to FR
      m_frontLeftModule = new SwerveModuleIOSim(DriveMap.FrontLeftSwerveModule);
      m_frontRightModule = new SwerveModuleIOSim(DriveMap.FrontRightSwerveModule);
      m_rearLeftModule = new SwerveModuleIOSim(DriveMap.RearLeftSwerveModule);
      m_rearRightModule = new SwerveModuleIOSim(DriveMap.RearRightSwerveModule);

      // Create pose estimator
      m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
          Rotation2d.fromDegrees(m_gyroSim.getAngle()), getModulePositions(), new Pose2d());
    }

    // Configure snap-to PID
    m_snapAngleController = DriveMap.SnapToPID.createPIDController(0.02);
    m_snapAngleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public DrivetrainInputs getInputs() {
    m_moduleInputs[0] = m_frontLeftModule.getInputs();
    m_moduleInputs[1] = m_frontRightModule.getInputs();
    m_moduleInputs[2] = m_rearLeftModule.getInputs();
    m_moduleInputs[3] = m_rearRightModule.getInputs();
    _inputs.ModuleStates = getModuleStates();

    _inputs.RobotRelativeChassisSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
    if (_robotIsReal) {
      _inputs.GyroAngle = m_gyro.getRotation2d();
      _inputs.GyroAccelX = m_gyro.getAccelerationX().getValueAsDouble();
      _inputs.GyroAccelY = m_gyro.getAccelerationY().getValueAsDouble();
      _inputs.GyroAccelZ = m_gyro.getAccelerationZ().getValueAsDouble();
    } else {
      m_gyroSim.setAngle(new Rotation2d(Rotation2d.fromDegrees(m_gyroSim.getAngle()).getRadians()
          + _inputs.RobotRelativeChassisSpeeds.omegaRadiansPerSecond * 0.02).getDegrees());
      _inputs.GyroAngle = Rotation2d.fromDegrees(m_gyroSim.getAngle());
    }

    var modulePositions = getModulePositions();
    _inputs.EstimatedRobotPose = m_poseEstimator.update(_inputs.GyroAngle, modulePositions);

    return _inputs;
  }

  public void setOutputs(DrivetrainOutputs outputs) {
    _outputs = outputs;

    if (_outputs.SnapEnabled) {
      if (_robotIsReal) {
        m_snapAngleController.setSetpoint(_outputs.SnapSetpoint.getRadians());
      } else {
        // Set GyroSim to the snap angle with no simulated delays.
        m_gyroSim.setAngle(outputs.SnapSetpoint.getDegrees());
      }
    }

    switch (_outputs.ControlMode) {
      case kRobotRelative:
        driveRobotRelative(_outputs.DesiredChassisSpeeds, _outputs.SnapEnabled);
        break;
      case kFieldRelative:
        drivePathPlanner(_outputs.DesiredChassisSpeeds);
        break;
      default:
        break;
    }

    m_frontLeftModule.setOutputs(m_moduleOutputs[0]);
    m_frontRightModule.setOutputs(m_moduleOutputs[1]);
    m_rearLeftModule.setOutputs(m_moduleOutputs[2]);
    m_rearRightModule.setOutputs(m_moduleOutputs[3]);
  }

  public void setDriveVoltages(double volts) {
    var angle = Rotation2d.fromDegrees(0);
    m_frontLeftModule.setDriveVoltage(volts, angle);
    m_frontRightModule.setDriveVoltage(volts, angle);
    m_rearLeftModule.setDriveVoltage(volts, angle);
    m_rearRightModule.setDriveVoltage(volts, angle);
  }

  public void logSysIdDrive(SysIdRoutineLog log) {
    // Record a frame for the left motors. Since these share an encoder, we consider
    // the entire group to be one motor.
    log.motor("Front-Left-Module").voltage(Units.Volts.of(m_moduleInputs[0].DriveMotorVoltage)) // measured motor voltage
        .linearPosition(Units.Meters.of(m_moduleInputs[0].ModulePosition.distanceMeters)) // distance in meters
        .linearVelocity(
            Units.MetersPerSecond.of(m_moduleInputs[0].ModuleState.speedMetersPerSecond)); // speed in
                                                                                                           // meters per
                                                                                                           // second

  }

  public void resetGyro() {
    if (_robotIsReal) {
      m_gyro.setYaw(Robot.onBlueAlliance() ? 180 : 0);

      m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(),
          m_poseEstimator.getEstimatedPosition());
    } else {
      m_gyroSim.setAngle(Robot.onBlueAlliance() ? 180 : 0);

      m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_gyroSim.getAngle()),
          getModulePositions(), m_poseEstimator.getEstimatedPosition());
    }
  }

  public void setEstimatorPose(Pose2d pose) {
    if (_robotIsReal) {
      m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    } else {
      m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_gyroSim.getAngle()),
          getModulePositions(), pose);
    }
  }

  public void addPoseEstimatorVisionMeasurement(Pose2d pose, double timestamp,
      Matrix<N3, N1> stdDeviations) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp, stdDeviations);
  }

  public void stopAllMotors() {
    m_frontLeftModule.stopMotors();
    m_frontRightModule.stopMotors();
    m_rearLeftModule.stopMotors();
    m_rearRightModule.stopMotors();
  }

  /**
   * Drives robot-relative using a ChassisSpeeds
   * 
   * @param desiredChassisSpeeds The desired speeds of the robot
   */
  private void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean snapAngleEnabled) {
    // If snap-to is enabled, calculate and override the input rotational speed to
    // reach the setpoint
    if (snapAngleEnabled) {
      if (_robotIsReal) {
        var currentRotationRadians = MathUtil.angleModulus(m_gyro.getRotation2d().getRadians());
        var snapCorrection = m_snapAngleController.calculate(currentRotationRadians);
        _inputs.SnapCorrectionRadiansPerSecond = snapCorrection;
        desiredChassisSpeeds.omegaRadiansPerSecond = snapCorrection;

        // Report back if snap is on-target
        _inputs.SnapIsOnTarget = Math.abs(desiredChassisSpeeds.omegaRadiansPerSecond) < 0.1;
      } else {
        // Report back that snap is on-target since we're assuming there is no loss in
        // the simulation
        _inputs.SnapIsOnTarget = true;
      }
    }

    // Correct drift by taking the input speeds and converting them to a desired
    // per-period speed. This is known as "discretizing"
    desiredChassisSpeeds = ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);

    // Calculate the module states from the chassis speeds
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        DriveMap.MaxSpeedMetersPerSecond);

    // Set the desired states for each module
    setDesiredModuleStates(swerveModuleStates);
  }

  /**
   * Facilitates driving using PathPlanner generated speeds
   * 
   * @param robotRelativeSpeeds
   */
  private void drivePathPlanner(ChassisSpeeds robotRelativeSpeeds) {
    if (Robot.onRedAlliance()) {
      Rotation2d gyroAngle = _robotIsReal ? m_gyro.getRotation2d().plus(Rotation2d.fromDegrees(180))
          : Rotation2d.fromDegrees(m_gyroSim.getAngle()).plus(Rotation2d.fromDegrees(180));

      // Convert the robot-relative speeds to field-relative speeds with the flipped
      // gyro
      var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, gyroAngle);

      // Convert back to robot-relative speeds, also with the flipped gyro
      robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, gyroAngle);

      driveRobotRelative(robotRelativeSpeeds, false);
    } else {
      driveRobotRelative(robotRelativeSpeeds, false);
    }
  }

  /**
   * Sets the desired states for each swerve module in order FL, FR, RL, RR
   * 
   * @param desiredStates
   */
  private void setDesiredModuleStates(SwerveModuleState[] desiredStates) {
    m_moduleOutputs[0].DesiredState = desiredStates[0];
    m_moduleOutputs[1].DesiredState = desiredStates[1];
    m_moduleOutputs[2].DesiredState = desiredStates[2];
    m_moduleOutputs[3].DesiredState = desiredStates[3];
  }

  /**
   * Gets the instantaneous states for each swerve module in FL, FR, RL, RR order
   */
  @Logged(importance = Importance.CRITICAL)
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] { m_moduleInputs[0].ModuleState, m_moduleInputs[1].ModuleState,
        m_moduleInputs[2].ModuleState, m_moduleInputs[3].ModuleState };
  }

  @Logged(importance = Importance.CRITICAL)
  public SwerveModuleState[] getDesiredModuleStates() {
    return new SwerveModuleState[] {
        m_moduleOutputs[0].DesiredState, m_moduleOutputs[1].DesiredState, m_moduleOutputs[2].DesiredState,
        m_moduleOutputs[3].DesiredState
    };
  }

  /**
   * Gets the cumulative positions for each swerve module in FL, FR, RL, RR order
   */
  @Logged(importance = Importance.CRITICAL)
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { m_moduleInputs[0].ModulePosition,
        m_moduleInputs[1].ModulePosition, m_moduleInputs[2].ModulePosition,
        m_moduleInputs[3].ModulePosition, };
  }
}
