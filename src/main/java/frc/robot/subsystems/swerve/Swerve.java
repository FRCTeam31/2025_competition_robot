package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.dashboard.DrivetrainDashboardSection;
import frc.robot.game.AprilTagReefMap;
import frc.robot.game.ReefBranchSide;
import frc.robot.oi.ImpactRumbleHelper;
import frc.robot.Container;
import frc.robot.Elastic;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.util.AutoAlign;
import frc.robot.subsystems.vision.LimelightNameEnum;
import frc.robot.subsystems.vision.Vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.PrimeHolonomicDriveController;
import org.prime.control.SwerveControlSuppliers;
import org.prime.pose.PoseUtil;
import org.prime.vision.LimelightInputs;

public class Swerve extends SubsystemBase {

  private static final InterpolatingDoubleTreeMap DriveSpeedSlowCoeffient = InterpolatingDoubleTreeMap
      .ofEntries(
          Map.entry(0.3135d, 0.9d),
          Map.entry(0.35d, 0.8d),
          Map.entry(0.39d, 0.7d),
          Map.entry(0.43d, 0.6d),
          Map.entry(0.47d, 0.5d),
          Map.entry(0.51d, 0.4d),
          Map.entry(0.55d, 0.3d),
          Map.entry(0.6d, 0.2d));
  private DrivetrainDashboardSection _drivetrainDashboardSection;
  private ImpactRumbleHelper _rumbleHelper;
  private SwerveControlSuppliers _controlSuppliers;

  // IO
  private SwerveIOPackager _swervePackager;
  private SwerveSubsystemInputsAutoLogged _inputs = new SwerveSubsystemInputsAutoLogged();

  // AutoAlign & Pathfinding
  private AutoAlign _autoAlign;
  private Command _activePathfindCommand;

  // Vision, Kinematics, odometry
  private PrimeHolonomicDriveController _primeHolonomicController;
  private RobotConfig _pathplannerRobotConfig;

  /**
   * Creates a new Drivetrain.
   */
  public Swerve(boolean isReal) {
    setName("Swerve");

    _rumbleHelper = new ImpactRumbleHelper();

    // Create swerve controller
    _swervePackager = new SwerveIOPackager(isReal);
    _swervePackager.updateInputs(_inputs);

    // Configure AutoAlign
    _autoAlign = new AutoAlign(SwerveMap.AutoAlignPID);

    _drivetrainDashboardSection = new DrivetrainDashboardSection();

    configurePathPlanner();
  }

  private void configurePathPlanner() {
    // Load the RobotConfig from the GUI settings, or use the default if an exception occurs
    _pathplannerRobotConfig = SwerveMap.PathPlannerRobotConfiguration;
    try {
      _pathplannerRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Set up PP to feed current path poses to the dashboard's field widget
    // PathPlannerLogging.setLogCurrentPoseCallback(pose -> Container.TeleopDashboardSection.setFieldRobotPose(pose));
    PathPlannerLogging
        .setLogTargetPoseCallback(pose -> Container.TeleopDashboardSection.getFieldTargetPose().setPose(pose));
    PathPlannerLogging
        .setLogActivePathCallback(poses -> Container.TeleopDashboardSection.getFieldPath().setPoses(poses));

    // Configure PathPlanner holonomic control
    _primeHolonomicController = new PrimeHolonomicDriveController(
        SwerveMap.PathPlannerTranslationPID.toPIDConstants(),
        SwerveMap.PathPlannerRotationPID.toPIDConstants());
    AutoBuilder.configure(
        () -> _inputs.EstimatedRobotPose,
        _swervePackager::setEstimatorPose,
        () -> _inputs.RobotRelativeChassisSpeeds,
        (speeds, feedForwards) -> driveRobotRelative(speeds),
        _primeHolonomicController,
        _pathplannerRobotConfig,
        Robot::onRedAlliance, // Boolean supplier that controls when the path will be mirrored for the red alliance
        this);

    // Override PathPlanner's rotation feedback
    // PPHolonomicDriveController.overrideRotationFeedback(() -> _inputs.AutoAlignCorrection);
  }

  // #region Control methods

  public void setControlProfile(SwerveControlSuppliers controlSuppliers) {
    _controlSuppliers = controlSuppliers;
  }

  /**
   * Resets the gyro angle
   */
  public void resetGyro() {
    _swervePackager.resetGyro();
  }

  /**
   * Enabled/disables AutoAlign control. Also overrides PathPlanner's rotation, if enabled
   */
  private void setAutoAlignEnabled(boolean enabled) {
    _inputs.UseAutoAlign = enabled;
    if (enabled) {
      PPHolonomicDriveController.overrideRotationFeedback(() -> _inputs.AutoAlignCorrection);
    } else {
      PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
  }

  /**
   * Drives robot-relative using a ChassisSpeeds
   * 
   * @param robotRelativeChassisSpeeds The desired speeds of the robot
   */
  private void driveRobotRelative(ChassisSpeeds robotRelativeChassisSpeeds) {
    // If AutoAlign is enabled, override the input rotational speed to reach the setpoint
    Logger.recordOutput(getName() + "/autoAlignCorrection", _inputs.AutoAlignCorrection);

    robotRelativeChassisSpeeds.omegaRadiansPerSecond = _inputs.UseAutoAlign
        ? _inputs.AutoAlignCorrection
        : robotRelativeChassisSpeeds.omegaRadiansPerSecond;

    // Correct drift by taking the input speeds and converting them to a desired
    // per-period speed. This is known as "discretizing"
    robotRelativeChassisSpeeds = ChassisSpeeds.discretize(robotRelativeChassisSpeeds, 0.02);
    Logger.recordOutput(getName() + "/desiredChassisSpeeds", robotRelativeChassisSpeeds);

    if (DriverStation.isTeleopEnabled()) {
      double elevatorHeight = Container.Elevator.getElevatorPositionMeters();
      double speedCoef = DriveSpeedSlowCoeffient.get(elevatorHeight);
      speedCoef = elevatorHeight < 0.3 ? 1 : speedCoef;

      robotRelativeChassisSpeeds.vxMetersPerSecond = robotRelativeChassisSpeeds.vxMetersPerSecond * speedCoef;
      robotRelativeChassisSpeeds.vyMetersPerSecond = robotRelativeChassisSpeeds.vyMetersPerSecond * speedCoef;
      robotRelativeChassisSpeeds.omegaRadiansPerSecond = robotRelativeChassisSpeeds.omegaRadiansPerSecond * speedCoef;
    }

    // Calculate the module states from the chassis speeds
    var swerveModuleStates = _swervePackager.Kinematics.toSwerveModuleStates(robotRelativeChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveMap.Chassis.MaxSpeedMetersPerSecond);

    // Set the desired states for each module
    Logger.recordOutput(getName() + "/desiredStates", swerveModuleStates);
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

    var withinPoseEstimationVelocity = currentRotationalVelocity.lt(DegreesPerSecond.of(60))
        && currentXVelocity.lt(MetersPerSecond.of(2))
        && currentYVelocity.lt(MetersPerSecond.of(2));

    Logger.recordOutput(getName() + "/withinPoseEstimationVelocity", withinPoseEstimationVelocity);
    if (!withinPoseEstimationVelocity) {
      return;
    }

    evaluatePoseEstimation(Container.Vision.getLimelightInputs(LimelightNameEnum.kFront));
    evaluatePoseEstimation(Container.Vision.getLimelightInputs(LimelightNameEnum.kRear));

    var llInputs = Container.Vision.getLimelightInputs(LimelightNameEnum.kFront);

    if (!Vision.isReefTag(llInputs.ApriltagId)) {
      return;
    }

    var reefSide = AprilTagReefMap.getReefSide(llInputs.ApriltagId);
    Logger.recordOutput(getName() + "/driveToInViewReefTargetBranch/targeted-face", reefSide.getFaceName());

    // Get the target pose, and convert it to field space
    Logger.recordOutput(getName() + "/driveToInViewReefTargetBranch/target-pose-robot-space",
        llInputs.RobotSpaceTargetPose.Pose.toPose2d());
    var aprilTagPoseFieldSpace = PoseUtil.convertPoseFromRobotToFieldSpace(
        _inputs.EstimatedRobotPose,
        llInputs.RobotSpaceTargetPose.Pose.toPose2d());
    Logger.recordOutput(getName() + "/driveToInViewReefTargetBranch/target-pose-field-space", aprilTagPoseFieldSpace);

    // Get the approach pose for the desired branch side
    var leftApproachPose = AprilTagReefMap.getBranchPoseFromTarget(
        ReefBranchSide.kLeft,
        aprilTagPoseFieldSpace);
    var rightApproachPose = AprilTagReefMap.getBranchPoseFromTarget(
        ReefBranchSide.kRight,
        aprilTagPoseFieldSpace);
    Logger.recordOutput(getName() + "/driveToInViewReefTargetBranch/branch-approach-pose-L", leftApproachPose);
    Logger.recordOutput(getName() + "/driveToInViewReefTargetBranch/branch-approach-pose-R", rightApproachPose);
  }

  /**
   * Evaluates a limelight pose and feeds it into the pose estimator
   */
  private void evaluatePoseEstimation(LimelightInputs limelightInputs) {
    // If we have a valid target, update the pose estimator
    if (!Vision.isAprilTagIdValid(limelightInputs.ApriltagId))
      return;

    var llPose = limelightInputs.BlueAllianceOriginFieldSpaceRobotPose;

    // if (_inputs.EstimatedRobotPose.getTranslation().getDistance(llPose.Pose.toPose2d().getTranslation()) <= 1) {
    _swervePackager.addPoseEstimatorVisionMeasurement(
        llPose.Pose.toPose2d(),
        llPose.Timestamp,
        llPose.getStdDeviations());
    // }
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
    Container.TeleopDashboardSection.setFieldRobotPose(_inputs.EstimatedRobotPose);

    // Update LEDs
    Logger.recordOutput(getName() + "/autoAlign/Enabled", _inputs.UseAutoAlign);
    Logger.recordOutput(getName() + "/autoAlign/Setpoint", _autoAlign.getSetpoint());
    Logger.recordOutput(getName() + "/autoAlign/AtSetpoint", _autoAlign.atSetpoint());

    if (DriverStation.isAutonomousEnabled()) {
      Logger.recordOutput(getName() + "/pp-translation-error", _primeHolonomicController.getTranslationError());
    }

    // Update shuffleboard
    if (DriverStation.isEnabled()) {
      Container.TeleopDashboardSection.setFieldRobotPose(_inputs.EstimatedRobotPose);
      Container.TeleopDashboardSection.setGyroHeading(_inputs.GyroAngle);
    }
    Logger.recordOutput(getName() + "/estimatedRobotPose", _inputs.EstimatedRobotPose);
    _drivetrainDashboardSection.setAutoAlignEnabled(_inputs.UseAutoAlign);
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
   * @param controlSuppliers Controller input suppliers
   */
  public Command driveFieldRelativeCommand(SwerveControlSuppliers controlSuppliers) {
    return this.run(() -> {
      var speeds = controlSuppliers.getChassisSpeeds(
          false,
          _inputs.GyroAngle,
          () -> setAutoAlignEnabled(false));

      // If the driver is trying to move and we have an active pathfinding command, cancel it.
      var driverIsTryingToMove = speeds.omegaRadiansPerSecond > 0
          || speeds.vxMetersPerSecond > 0
          || speeds.vyMetersPerSecond > 0;
      if (driverIsTryingToMove) {
        var pathfindCommandIsCancellable = _activePathfindCommand != null
            && _activePathfindCommand.isScheduled()
            && !_activePathfindCommand.isFinished();

        if (pathfindCommandIsCancellable) {
          _activePathfindCommand.cancel();
        }
      }

      driveRobotRelative(speeds);
    });
  }

  /**
   * Creates a command that drives the robot to the desired branch of the in-view reef side
   * @param branchSide
   * @return
   */
  public Command driveToReefTargetBranch(ReefBranchSide branchSide, SwerveControlSuppliers controlSuppliers) {
    return stopAllMotorsCommand()
        .andThen(Commands.runOnce(() -> {
          // Cancel the command if it's already running
          if (_activePathfindCommand != null && _activePathfindCommand.isScheduled()
              && !_activePathfindCommand.isFinished()) {
            _activePathfindCommand.cancel();
          }
        }))
        .andThen(this.defer(() -> {
          var llInputs = Container.Vision.getLimelightInputs(LimelightNameEnum.kFront);

          if (!Vision.isReefTag(llInputs.ApriltagId)) {
            Container.Vision.blinkLed(LimelightNameEnum.kRear, 2);
            Elastic.sendWarning("Command Failed", "No reef tag in view");

            return Commands.print("[SWERVE] - No reef tag in view");
          }

          var reefSide = AprilTagReefMap.getReefSide(llInputs.ApriltagId);
          Logger.recordOutput(getName() + "/driveToReefTargetBranch/targeted-face", reefSide.getFaceName());
          Logger.recordOutput(getName() + "/driveToReefTargetBranch/targeted-branch",
              reefSide.getBranchName(branchSide));

          // Check to make sure the tag pose is available
          if (reefSide.TagPose.isEmpty()) {
            Container.Vision.blinkLed(LimelightNameEnum.kRear, 2);
            Elastic.sendWarning("Command Failed", "AprilTag pose not found in field layout");

            return Commands.print("[SWERVE] - AprilTag " + llInputs.ApriltagId + " pose not found in field layout");
          }

          var targetPose = reefSide.TagPose.orElseThrow();
          Logger.recordOutput(getName() + "/driveToReefTargetBranch/target-pose-field-space", targetPose);

          // Get the approach pose for the desired branch side
          var branchPose = AprilTagReefMap.getBranchPoseFromTarget(branchSide, targetPose.toPose2d()); // translated over 16.5 cm
          var approachPose = AprilTagReefMap.getApproachPose(branchPose, SwerveMap.Chassis.ApproachDistance); // translated forward
          Logger.recordOutput(getName() + "/driveToReefTargetBranch/branch-approach-pose", approachPose);

          var pathfindConstraints = new PathConstraints(
              SwerveMap.Chassis.MaxSpeedMetersPerSecond,
              SwerveMap.Chassis.MaxSpeedMetersPerSecond * 1.5,
              SwerveMap.Chassis.MaxAngularSpeedRadians,
              SwerveMap.Chassis.MaxAngularSpeedRadians * 1.5);

          _activePathfindCommand = AutoBuilder.pathfindToPose(approachPose, pathfindConstraints)
              .withTimeout(3); // TODO: Check this

          return _activePathfindCommand;
        }));
    // .andThen(setAutoAlignSetpointCommand(_inputs.GyroAngle.getDegrees()));
  }

  /**
   * Command for stopping all motors
   */
  public Command stopAllMotorsCommand() {
    return this.runOnce(_swervePackager::stopAllMotors);
  }

  /**
   * Command for resetting the gyro
   */
  public Command resetGyroCommand() {
    return Commands.runOnce(_swervePackager::resetGyro);
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

  /**
   * Creates a command which pathfinds to a given pose, flipping the path across the field center if desired
   * @param poseSupplier A supplier for the target pose
   */
  public Command pathfindToPoseCommand(Supplier<Pose2d> poseSupplier, boolean flipped) {
    return this.defer(() -> {
      var pathConstraints = new PathConstraints(SwerveMap.Chassis.MaxSpeedMetersPerSecond,
          SwerveMap.Chassis.MaxSpeedMetersPerSecond,
          SwerveMap.Chassis.MaxAngularSpeedRadians,
          SwerveMap.Chassis.MaxAngularSpeedRadians);

      var desiredPose = poseSupplier.get();

      return flipped
          ? AutoBuilder.pathfindToPoseFlipped(desiredPose, pathConstraints)
          : AutoBuilder.pathfindToPose(desiredPose, pathConstraints);
    });
  }

  /*
   * Returns a map of named commands for the drivetrain subsystem for PathPlanner
   */
  public Map<String, Command> getNamedCommands() {
    return Map.of();
  }
  // #endregion
}
