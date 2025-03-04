package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.dashboard.DrivetrainDashboardSection;
import frc.robot.oi.ImpactRumbleHelper;
import frc.robot.Container;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.util.AutoAlign;
import frc.robot.subsystems.vision.VisionSubsystem;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.prime.control.SwerveControlSuppliers;
import org.prime.vision.LimelightInputs;

public class SwerveSubsystem extends SubsystemBase {
  private DrivetrainDashboardSection _drivetrainDashboardSection;
  private ImpactRumbleHelper _rumbleHelper;

  // IO
  private SwerveIOPackager _swervePackager;
  private SwerveSubsystemInputsAutoLogged _inputs = new SwerveSubsystemInputsAutoLogged();

  // AutoAlign
  private boolean _useAutoAlign = false;
  private AutoAlign _autoAlign;

  // Vision, Kinematics, odometry
  public boolean WithinPoseEstimationVelocity = true;

  private LEDPattern _alignOnTargetPattern = LEDPattern
      .solid(Color.kGreen)
      .blink(Units.Seconds.of(0.1));
  private LEDPattern _alignOffTargetPattern = LEDPattern
      .steps(Map.of(0.0, Color.kRed, 0.25, Color.kBlack))
      .scrollAtRelativeSpeed(Units.Hertz.of(2));

  SysIdRoutine _driveSysIdRoutine;

  /**
   * Creates a new Drivetrain.
   */
  public SwerveSubsystem(boolean isReal) {
    setName("Drivetrain");

    _rumbleHelper = new ImpactRumbleHelper();

    // Create swerve controller
    _swervePackager = new SwerveIOPackager(isReal);
    _swervePackager.updateInputs(_inputs);

    // Configure AutoAlign
    _autoAlign = new AutoAlign(SwerveMap.AutoAlignPID);

    _drivetrainDashboardSection = new DrivetrainDashboardSection();

    configurePathPlanner();

    // Create a new SysId routine for characterizing the drive.
    // TODO: Remove when no longer needed
    _driveSysIdRoutine = new SysIdRoutine(
        // Ramp up at 1 volt per second for quasistatic tests, step at 2 volts in
        // dynamic tests, run for 13 seconds.
        new SysIdRoutine.Config(Units.Volts.of(2).per(Units.Second), Units.Volts.of(8),
            Units.Seconds.of(15)),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            _swervePackager::setDriveVoltages,
            // Tell SysId how to record a frame of data for each motor on the mechanism
            // being characterized.
            _swervePackager::logSysIdDrive,
            // Tell SysId to make generated commands require this subsystem, suffix test
            // state in WPILog with this subsystem's name
            this));
  }

  public Command driveSwerveVoltage(double voltage) {
    return Commands.runOnce(() -> {
      _swervePackager.setDriveVoltages(Units.Volts.of(voltage));
    }, this);
  }

  private void configurePathPlanner() {
    // Load the RobotConfig from the GUI settings, or use the default if an
    // exception occurs
    RobotConfig config = SwerveMap.PathPlannerRobotConfiguration;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Set up PP to feed current path poses to the dashboard's field widget
    PathPlannerLogging.setLogCurrentPoseCallback(pose -> Container.TeleopDashboardSection.setFieldRobotPose(pose));
    PathPlannerLogging
        .setLogTargetPoseCallback(pose -> Container.TeleopDashboardSection.getFieldTargetPose().setPose(pose));
    PathPlannerLogging
        .setLogActivePathCallback(poses -> Container.TeleopDashboardSection.getFieldPath().setPoses(poses));

    // Configure PathPlanner holonomic control
    AutoBuilder.configure(
        () -> _inputs.EstimatedRobotPose,
        _swervePackager::setEstimatorPose,
        () -> _inputs.RobotRelativeChassisSpeeds,
        (speeds, feedForwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            SwerveMap.PathPlannerTranslationPID.toPIDConstants(),
            SwerveMap.PathPlannerRotationPID.toPIDConstants()),
        config, () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          var alliance = DriverStation.getAlliance();

          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        }, this);

    // Override PathPlanner's rotation feedback
    // PPHolonomicDriveController.overrideRotationFeedback(() -> _inputs.AutoAlignCorrection);

  }

  // #region Control methods

  /**
   * Resets the gyro angle
   */
  public void resetGyro() {
    _swervePackager.resetGyro();
  }

  /**
   * Enabled/disables AutoAlign control
   */
  private void setAutoAlignEnabled(boolean enabled) {
    _useAutoAlign = enabled;
    if (!enabled)
      Container.LEDs.clearForegroundPattern();
  }

  /**
   * Sets the pose estimator's pose
   * @param pose
   */
  public void setEstimatorPose(Pose2d pose) {
    _swervePackager.setEstimatorPose(pose);
  }

  /**
   * Drives robot-relative using a ChassisSpeeds
   * 
   * @param robotRelativeChassisSpeeds The desired speeds of the robot
   */
  private void driveRobotRelative(ChassisSpeeds robotRelativeChassisSpeeds) {
    // If AutoAlign is enabled, override the input rotational speed to reach the setpoint
    Logger.recordOutput("Drive/autoAlignCorrection", _inputs.AutoAlignCorrection);

    robotRelativeChassisSpeeds.omegaRadiansPerSecond = _useAutoAlign
        ? _inputs.AutoAlignCorrection
        : robotRelativeChassisSpeeds.omegaRadiansPerSecond;

    // Correct drift by taking the input speeds and converting them to a desired
    // per-period speed. This is known as "discretizing"
    robotRelativeChassisSpeeds = ChassisSpeeds.discretize(robotRelativeChassisSpeeds, 0.02);
    Logger.recordOutput("Drive/desiredChassisSpeeds", robotRelativeChassisSpeeds);

    // Calculate the module states from the chassis speeds
    var swerveModuleStates = _swervePackager.Kinematics.toSwerveModuleStates(robotRelativeChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveMap.Chassis.MaxSpeedMetersPerSecond);

    // Set the desired states for each module
    Logger.recordOutput("Drive/desiredStates", swerveModuleStates);
    _swervePackager.setDesiredModuleStates(swerveModuleStates);

    // Update the gyro omega for simulation purposes
    _swervePackager.setSimGyroOmega(robotRelativeChassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Processes vision estimations when within a certain velocity threshold
   */
  private void processVisionEstimations() {
    // (1 rad/s is about 60 degrees/s)
    var currentRotationalVelocity = RadiansPerSecond
        .of(Math.abs(_inputs.RobotRelativeChassisSpeeds.omegaRadiansPerSecond));
    var currentXVelocity = MetersPerSecond.of(_inputs.RobotRelativeChassisSpeeds.vxMetersPerSecond);
    var currentYVelocity = MetersPerSecond.of(_inputs.RobotRelativeChassisSpeeds.vyMetersPerSecond);

    WithinPoseEstimationVelocity = currentRotationalVelocity.lt(DegreesPerSecond.of(60))
        && currentXVelocity.lt(MetersPerSecond.of(2))
        && currentYVelocity.lt(MetersPerSecond.of(2));

    if (!WithinPoseEstimationVelocity) {
      return;
    }

    var limelightInputs = Container.Vision.getAllLimelightInputs();

    if (Container.TeleopDashboardSection.getFrontPoseEstimationSwitch())
      evaluatePoseEstimation(limelightInputs[0]);

    if (Container.TeleopDashboardSection.getRearPoseEstimationSwitch())
      evaluatePoseEstimation(limelightInputs[1]);
  }

  /**
   * Evaluates a limelight pose and feeds it into the pose estimator
   */
  private void evaluatePoseEstimation(LimelightInputs limelightInputs) {
    // If we have a valid target, update the pose estimator
    if (!VisionSubsystem.isAprilTagIdValid(limelightInputs.ApriltagId))
      return;

    var llPose = Robot.onBlueAlliance()
        ? limelightInputs.BlueAllianceOriginFieldSpaceRobotPose
        : limelightInputs.RedAllianceOriginFieldSpaceRobotPose;

    _swervePackager.addPoseEstimatorVisionMeasurement(
        llPose.Pose.toPose2d(),
        llPose.Timestamp,
        llPose.getStdDeviations());
  }

  // #endregion

  /**
   * Updates odometry and any other periodic drivetrain events
   */
  @Override
  public void periodic() {
    // Get inputs
    _swervePackager.updateInputs(_inputs);
    _inputs.AutoAlignCorrection = _autoAlign.getCorrection(_inputs.GyroAngle);
    Logger.processInputs(getName(), _inputs);

    processVisionEstimations();

    // Update LEDs
    Logger.recordOutput("Drive/autoAlignEnabled", _useAutoAlign);
    Logger.recordOutput("Drive/autoAlignSetpoint", _autoAlign.getSetpoint());
    Logger.recordOutput("Drive/autoAlignAtSetpoint", _autoAlign.atSetpoint());
    if (_useAutoAlign) {
      Container.LEDs.setForegroundPattern(_autoAlign.atSetpoint()
          ? _alignOnTargetPattern
          : _alignOffTargetPattern);
    }

    // Update shuffleboard
    if (DriverStation.isEnabled()) {
      Container.TeleopDashboardSection.setFieldRobotPose(_inputs.EstimatedRobotPose);
      Container.TeleopDashboardSection.setGyroHeading(_inputs.GyroAngle);
    }
    Logger.recordOutput("Drive/estimatedRobotPose", _inputs.EstimatedRobotPose);
    _drivetrainDashboardSection.setAutoAlignEnabled(_useAutoAlign);
    _drivetrainDashboardSection.setAutoAlignTarget(_autoAlign.getSetpoint());

    // Update rumble
    _rumbleHelper.addSample(
        _inputs.GyroAccelX,
        _inputs.GyroAccelY,
        _inputs.GyroAccelZ,
        _inputs.RobotRelativeChassisSpeeds.vxMetersPerSecond,
        SwerveMap.Chassis.MaxSpeedMetersPerSecond);
    Container.OperatorInterface.setDriverRumbleIntensity(_rumbleHelper.getRumbleIntensity());
  }

  // #region Commands

  /**
   * Creates a command that drives the robot in field-relative mode using input controls
   * 
   * @param controlSuppliers Controller input suppliers
   */
  public Command driveFieldRelativeCommand(SwerveControlSuppliers controlSuppliers) {
    return this.run(() -> driveRobotRelative(controlSuppliers.getChassisSpeeds(false, _inputs.GyroAngle,
        () -> setAutoAlignEnabled(false))));
  }

  /**
   * Creates a command that drives the robot in robot-relative mode using input controls
   * 
   * @param controlSuppliers Controller input suppliers
   */
  public Command driveRobotRelativeCommand(SwerveControlSuppliers controlSuppliers) {
    return this.run(() -> driveRobotRelative(controlSuppliers.getChassisSpeeds(true, _inputs.GyroAngle,
        () -> setAutoAlignEnabled(false))));
  }

  /**
   * Command for stopping all motors
   */
  public Command stopAllMotors() {
    return this.runOnce(() -> _swervePackager.stopAllMotors());
  }

  /**
   * Command for resetting the gyro
   */
  public Command resetGyroCommand() {
    return Commands.runOnce(() -> _swervePackager.resetGyro());
  }

  /**
   * Enables AutoAlign control and sets an angle setpoint
   * 
   * @param angle
   */
  public Command setAutoAlignSetpointCommand(double angle) {
    return Commands.runOnce(() -> {
      var setpoint = Robot.onBlueAlliance()
          ? angle + 180
          : angle;
      _autoAlign.setSetpoint(Rotation2d.fromDegrees(setpoint));
      setAutoAlignEnabled(true);
    });
  }

  /**
   * Disables AutoAlign control
   */
  public Command disableAutoAlignCommand() {
    var cmd = Commands.runOnce(() -> setAutoAlignEnabled(false));
    cmd.setName("DisableAutoAlign");

    return cmd;
  }

  /**
   * Enables lock-on control to whichever target is in view
   */
  public Command enableLockOnCommand() {
    var cmd = Commands.run(() -> {
      var rearLimelightInputs = Container.Vision.getLimelightInputs(1);

      // If targeted AprilTag is in validTargets, align to its offset
      if (VisionSubsystem.isAprilTagIdValid(rearLimelightInputs.ApriltagId)) {
        // Calculate the target heading
        var horizontalOffsetDeg = rearLimelightInputs.TargetHorizontalOffset.getDegrees();
        var robotHeadingDeg = _inputs.GyroAngle.getDegrees();
        var targetHeadingDeg = robotHeadingDeg - horizontalOffsetDeg;

        // Set the drivetrain to align to the target heading
        _autoAlign.setSetpoint(Rotation2d.fromDegrees(targetHeadingDeg));
        setAutoAlignEnabled(true);
      } else {
        setAutoAlignEnabled(false);
      }
    });

    cmd.setName("EnableLockOn");

    return cmd;
  }

  /**
   * Enables AutoAlign for PathPlanner routines
   * @return
   */
  public Command enablePathPlannerAutoAlignRotationFeedbackCommand() {
    var cmd = Commands.runOnce(() -> {
      PPHolonomicDriveController.overrideRotationFeedback(() -> _inputs.AutoAlignCorrection);
    });
    cmd.setName("EnableAutoAlignRotationFeedback");

    return cmd;
  }

  /**
   * Disables AutoAlign for PathPlanner routines
   * @return
   */
  public Command disablePathPlannerAutoAlignRotationFeedbackCommand() {
    var cmd = Commands.runOnce(PPHolonomicDriveController::clearRotationFeedbackOverride);
    cmd.setName("DisableAutoAlignRotationFeedback");

    return cmd;
  }

  public Command pathfindToPoseCommand(Pose2d pose) {
    var pathConstraints = new PathConstraints(SwerveMap.Chassis.MaxSpeedMetersPerSecond / 2,
        SwerveMap.Chassis.MaxSpeedMetersPerSecond / 4,
        SwerveMap.Chassis.MaxAngularSpeedRadians,
        SwerveMap.Chassis.MaxAngularSpeedRadians / 2);

    return AutoBuilder.pathfindToPoseFlipped(pose, pathConstraints);
  }

  // TODO: Remove when no longer needed
  public Command runSysIdQuasistaticRoutineCommand(Direction dir) {
    return _driveSysIdRoutine.quasistatic(dir);
  }

  // TODO: Remove when no longer needed
  public Command runSysIdDynamicRoutineCommand(Direction dir) {
    return _driveSysIdRoutine.dynamic(dir);
  }

  /*
   * Returns a map of named commands for the drivetrain subsystem for PathPlanner
   */
  public Map<String, Command> getNamedCommands() {
    return Map.of(
        "EnableTargetLockOn",
        enableLockOnCommand(),
        "DisableAutoAlign",
        disableAutoAlignCommand(),
        "EnableAutoAlignRotationFeedback",
        enablePathPlannerAutoAlignRotationFeedbackCommand(),
        "DisableAutoAlignRotationFeedback",
        disablePathPlannerAutoAlignRotationFeedbackCommand());
  }
  // #endregion
}
