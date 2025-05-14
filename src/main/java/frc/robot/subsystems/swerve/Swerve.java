package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.SuperStructure;
import frc.robot.subsystems.swerve.util.AutoAlign;
import frc.robot.subsystems.vision.LimelightInputs;
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

  // IO
  private SwerveIOPackager _swervePackager;

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
    _swervePackager.updateInputs(SuperStructure.SwerveState);

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
        () -> SuperStructure.SwerveState.EstimatedRobotPose,
        _swervePackager::setEstimatorPose,
        () -> SuperStructure.SwerveState.RobotRelativeChassisSpeeds,
        (speeds, feedForwards) -> driveRobotRelative(speeds),
        _primeHolonomicController,
        _pathplannerRobotConfig,
        Robot::onRedAlliance, // Boolean supplier that controls when the path will be mirrored for the red alliance
        this);

    // Override PathPlanner's rotation feedback
    // PPHolonomicDriveController.overrideRotationFeedback(() -> SuperStructure.SwerveState.AutoAlignCorrection);
  }

  // #region Control methods

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
    SuperStructure.SwerveState.UseAutoAlign = enabled;
    if (enabled) {
      PPHolonomicDriveController.overrideRotationFeedback(() -> SuperStructure.SwerveState.AutoAlignCorrection);
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
    Logger.recordOutput(getName() + "/autoAlignCorrection", SuperStructure.SwerveState.AutoAlignCorrection);

    robotRelativeChassisSpeeds.omegaRadiansPerSecond = SuperStructure.SwerveState.UseAutoAlign
        ? SuperStructure.SwerveState.AutoAlignCorrection
        : robotRelativeChassisSpeeds.omegaRadiansPerSecond;

    // Correct drift by taking the input speeds and converting them to a desired
    // per-period speed. This is known as "discretizing"
    robotRelativeChassisSpeeds = ChassisSpeeds.discretize(robotRelativeChassisSpeeds, 0.02);
    Logger.recordOutput(getName() + "/desiredChassisSpeeds", robotRelativeChassisSpeeds);

    var isRunningPathfind = _activePathfindCommand != null && _activePathfindCommand.isScheduled()
        && !_activePathfindCommand.isFinished();
    if (DriverStation.isTeleopEnabled() && !isRunningPathfind) {
      double elevatorHeight = SuperStructure.ElevatorState.DistanceMeters;
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
        .of(Math.abs(SuperStructure.SwerveState.RobotRelativeChassisSpeeds.omegaRadiansPerSecond));
    var currentXVelocity = MetersPerSecond.of(SuperStructure.SwerveState.RobotRelativeChassisSpeeds.vxMetersPerSecond);
    var currentYVelocity = MetersPerSecond.of(SuperStructure.SwerveState.RobotRelativeChassisSpeeds.vyMetersPerSecond);

    var withinPoseEstimationVelocity = currentRotationalVelocity.lt(DegreesPerSecond.of(60))
        && currentXVelocity.lt(MetersPerSecond.of(2))
        && currentYVelocity.lt(MetersPerSecond.of(2));

    Logger.recordOutput(getName() + "/withinPoseEstimationVelocity", withinPoseEstimationVelocity);
    if (!withinPoseEstimationVelocity) {
      return;
    }

    evaluatePoseEstimation(SuperStructure.LimelightStates.get(LimelightNameEnum.kFront));
    evaluatePoseEstimation(SuperStructure.LimelightStates.get(LimelightNameEnum.kRear));
  }

  /**
   * Evaluates a limelight pose and feeds it into the pose estimator
   */
  private void evaluatePoseEstimation(LimelightInputs llInputs) {
    // If no tags in view, reject the update
    if (llInputs.BotPoseEstimate.tagCount == 0)
      return;

    if (llInputs.BotPoseEstimate.tagCount == 1 && llInputs.BotPoseEstimate.rawFiducials.length == 1) {
      boolean isValidTarget = Vision.isAprilTagIdValid(llInputs.BotPoseEstimate.rawFiducials[0].id);
      boolean tooAmbiguous = llInputs.BotPoseEstimate.rawFiducials[0].ambiguity > .7;
      boolean tooFar = llInputs.BotPoseEstimate.rawFiducials[0].distToCamera > 3;

      // If the tag is not valid, too ambiguous, or too far, reject the update
      if (!isValidTarget || tooAmbiguous || tooFar)
        return;
    }

    // If we've made it this far, we can trust the pose estimate
    _swervePackager.addPoseEstimatorVisionMeasurement(
        llInputs.BotPoseEstimate.pose,
        llInputs.BotPoseEstimate.timestampSeconds,
        VecBuilder.fill(.5, .5, 9999999));
  }

  // #endregion

  /**
   * Updates odometry and any other periodic drivetrain events
   */
  @Override
  public void periodic() {
    // Get inputs
    _swervePackager.updateInputs(SuperStructure.SwerveState);
    SuperStructure.SwerveState.AutoAlignCorrection = _autoAlign.getCorrection(SuperStructure.SwerveState.GyroAngle);
    Logger.processInputs(getName(), SuperStructure.SwerveState);

    processVisionEstimations();
    Container.TeleopDashboardSection.setFieldRobotPose(SuperStructure.SwerveState.EstimatedRobotPose);

    // Update LEDs
    Logger.recordOutput(getName() + "/autoAlign/Enabled", SuperStructure.SwerveState.UseAutoAlign);
    Logger.recordOutput(getName() + "/autoAlign/Setpoint", _autoAlign.getSetpoint());
    Logger.recordOutput(getName() + "/autoAlign/AtSetpoint", _autoAlign.atSetpoint());

    if (DriverStation.isAutonomousEnabled()) {
      Logger.recordOutput(getName() + "/pp-translation-error", _primeHolonomicController.getTranslationError());
    }

    // Update shuffleboard
    if (DriverStation.isEnabled()) {
      Container.TeleopDashboardSection.setFieldRobotPose(SuperStructure.SwerveState.EstimatedRobotPose);
      Container.TeleopDashboardSection.setGyroHeading(SuperStructure.SwerveState.GyroAngle);
    }
    Logger.recordOutput(getName() + "/estimatedRobotPose", SuperStructure.SwerveState.EstimatedRobotPose);
    _drivetrainDashboardSection.setAutoAlignEnabled(SuperStructure.SwerveState.UseAutoAlign);
    _drivetrainDashboardSection.setAutoAlignTarget(_autoAlign.getSetpoint());

    // Update rumble
    _rumbleHelper.addSample(
        SuperStructure.SwerveState.GyroAccelX,
        SuperStructure.SwerveState.GyroAccelY,
        SuperStructure.SwerveState.GyroAccelZ,
        SuperStructure.SwerveState.RobotRelativeChassisSpeeds.vxMetersPerSecond,
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
          SuperStructure.SwerveState.GyroAngle,
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
    }).handleInterrupt(() -> DriverStation.reportWarning("[DRIVE] Default command interrupted", false));
  }

  public Command cancelPathfindingCommand() {
    return Commands.runOnce(() -> {
      // Cancel the command if it's already running
      if (_activePathfindCommand != null && _activePathfindCommand.isScheduled()
          && !_activePathfindCommand.isFinished()) {
        _activePathfindCommand.cancel();
      }
    });
  }

  /**
   * Creates a command that drives the robot to the desired branch of the in-view reef side
   * @param branchSide
   * @return
   */
  public Command driveToReefTargetBranch(ReefBranchSide branchSide, SwerveControlSuppliers controlSuppliers) {
    return this.defer(() -> {
      var llInputs = SuperStructure.LimelightStates.get(LimelightNameEnum.kFront);

      if (llInputs.CurrentResults.targets_Fiducials.length != 1 ||
          !Vision.isReefTag(llInputs.CurrentResults.targets_Fiducials[0].fiducialID)) {

        // TODO: figure out how to do this
        // Container.Vision.blinkLed(LimelightNameEnum.kRear, 2);
        Elastic.sendWarning("Command Failed", "No reef tag in view");

        return Commands.print("[SWERVE] - No reef tag in view");
      }

      var tagId = llInputs.CurrentResults.targets_Fiducials[0].fiducialID;

      var reefSide = AprilTagReefMap.getReefSide(tagId);
      Logger.recordOutput(getName() + "/driveToReefTargetBranch/targeted-face", reefSide.getFaceName());
      Logger.recordOutput(getName() + "/driveToReefTargetBranch/targeted-branch",
          reefSide.getBranchName(branchSide));

      // Check to make sure the tag pose is available
      if (reefSide.TagPose.isEmpty()) {
        // TODO: figure out how to do this
        // Container.Vision.blinkLed(LimelightNameEnum.kRear, 2);
        Elastic.sendWarning("Command Failed", "AprilTag pose not found in field layout");

        return Commands.print("[SWERVE] - AprilTag " + tagId + " pose not found in field layout");
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

      _activePathfindCommand = Commands.sequence(
          cancelPathfindingCommand(),
          stopAllMotorsCommand(),
          AutoBuilder.pathfindToPose(approachPose, pathfindConstraints)
              .withTimeout(5)
              .finallyDo(_swervePackager::stopAllMotors),
          stopAllMotorsCommand())
          .handleInterrupt(() -> DriverStation.reportWarning("[DRIVE] driveToReefTargetBranch interrupted", false))
          .asProxy();

      return _activePathfindCommand;
    });
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
      PPHolonomicDriveController.overrideRotationFeedback(() -> SuperStructure.SwerveState.AutoAlignCorrection);
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
