package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.dashboard.DrivetrainDashboardSection;
import frc.robot.game.AprilTagReefMap;
import frc.robot.game.ReefPegSide;
import frc.robot.oi.ImpactRumbleHelper;
import frc.robot.Container;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.util.AutoAlign;
import frc.robot.subsystems.vision.VisionSubsystem;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.prime.control.SwerveControlSuppliers;
import org.prime.pose.PoseUtil;
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

  /**
   * Creates a new Drivetrain.
   */
  public SwerveSubsystem(boolean isReal) {
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
    // PathPlannerLogging.setLogCurrentPoseCallback(pose -> Container.TeleopDashboardSection.setFieldRobotPose(pose));
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
   * Resets the gyro angle
   */
  public void resetGyroInverted() {
    _swervePackager.resetGyroInverted();
  }

  /**
   * Enabled/disables AutoAlign control
   */
  private void setAutoAlignEnabled(boolean enabled) {
    _useAutoAlign = enabled;
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

    evaluatePoseEstimation(limelightInputs[0]);
    evaluatePoseEstimation(limelightInputs[1]);

    // TESTING
    var frontInputs = limelightInputs[0];
    if (VisionSubsystem.isReefTag(frontInputs.ApriltagId)) {
      var reefBranch = AprilTagReefMap.getBranchPair(frontInputs.ApriltagId);
      Logger.recordOutput(getName() + "/reef-targetpose-side-name", reefBranch.getFaceName());

      // Get the target pose, and then get the L and R branch offsets from that
      var targetPose = frontInputs.RobotSpaceTargetPose.Pose;
      var leftPose = getReefPegsPoseFromInput(ReefPegSide.kLeft, targetPose.toPose2d());
      var rightPose = getReefPegsPoseFromInput(ReefPegSide.kRight, targetPose.toPose2d());

      Logger.recordOutput(getName() + "/reef-targetpose-left_branch", leftPose);
      Logger.recordOutput(getName() + "/reef-targetpose-right_branch", rightPose);

      // Generate a straight line trajectory from each side of the target pose
      var tConfig = new TrajectoryConfig(1, 1)
          .setStartVelocity(1)
          .setEndVelocity(1)
          .setKinematics(_swervePackager.Kinematics);
      var leftLineTraj = PoseUtil.generateStraightLineTrajectory(leftPose, 2, tConfig);
      var rightLineTraj = PoseUtil.generateStraightLineTrajectory(rightPose, 2, tConfig);

      // Get the closest pose in each trajectory to the robot's current pose
      var leftPoses = PoseUtil.getTrajectoryPoses(leftLineTraj);
      var rightPoses = PoseUtil.getTrajectoryPoses(rightLineTraj);

      var closestLeft = PoseUtil.getClosestPoseInList(_inputs.EstimatedRobotPose, leftPoses);
      var closestRight = PoseUtil.getClosestPoseInList(_inputs.EstimatedRobotPose, rightPoses);

      // If either of the closest poses are null, something went wrong
      if (closestLeft == null || closestRight == null) {
        Logger.recordOutput(getName() + "/reef-auto-side-selection", "FAILED");
        Logger.recordOutput(getName() + "/reef-auto-side-selection", "FAILED");
        return;
      }

      Logger.recordOutput(getName() + "/reef-left-traj-closest-pose", closestLeft);
      Logger.recordOutput(getName() + "/reef-right-traj-closest-pose", closestRight);

      // Get the distance to the closest pose in each trajectory
      var distanceToClosestLeft = PoseUtil.getDistanceBetweenPoses(_inputs.EstimatedRobotPose, closestLeft);
      var distanceToClosestRight = PoseUtil.getDistanceBetweenPoses(_inputs.EstimatedRobotPose, closestRight);

      Logger.recordOutput(getName() + "/reef-left-dist-to-closest-traj-pose", distanceToClosestLeft);
      Logger.recordOutput(getName() + "/reef-right-dist-to-closest-traj-pose", distanceToClosestRight);

      if (distanceToClosestLeft < distanceToClosestRight) {
        Logger.recordOutput(getName() + "/reef-auto-side-selection", "left");
      } else {
        Logger.recordOutput(getName() + "/reef-auto-side-selection", "right");
      }
    }
  }

  /**
   * Evaluates a limelight pose and feeds it into the pose estimator
   */
  private void evaluatePoseEstimation(LimelightInputs limelightInputs) {
    // If we have a valid target, update the pose estimator
    if (!VisionSubsystem.isAprilTagIdValid(limelightInputs.ApriltagId))
      return;

    var llPose = limelightInputs.BlueAllianceOriginFieldSpaceRobotPose;

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
    Container.TeleopDashboardSection.setFieldRobotPose(_inputs.EstimatedRobotPose);

    // Update LEDs
    Logger.recordOutput("Drive/autoAlignEnabled", _useAutoAlign);
    Logger.recordOutput("Drive/autoAlignSetpoint", _autoAlign.getSetpoint());
    Logger.recordOutput("Drive/autoAlignAtSetpoint", _autoAlign.atSetpoint());

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

  /**
   * Returns the pose of the desired side of reef pegs from the current robot pose (assumed to be the midpoint between pegs)
   */
  public Pose2d getReefPegsPoseFromInput(ReefPegSide side, Pose2d inputPose) {
    var currentHeadingRadians = _inputs.GyroAngle.getRadians();
    var a = side == ReefPegSide.kLeft
        ? currentHeadingRadians + (Math.PI / 2)
        : currentHeadingRadians - (Math.PI / 2);

    var newX = inputPose.getX() + Centimeters.mutable(16.5).in(Meters) * Math.cos(a);
    var newY = inputPose.getY() + Centimeters.mutable(16.5).in(Meters) * Math.sin(a);
    var desiredPose = new Pose2d(newX, newY, _inputs.GyroAngle);

    return desiredPose;
  }

  // #region Commands

  /**
   * Creates a command that drives the robot in field-relative mode using input controls
   * @param controlSuppliers Controller input suppliers
   */
  public Command driveFieldRelativeCommand(SwerveControlSuppliers controlSuppliers) {
    return this.run(() -> {
      if (_inputs.DrivingRobotRelative) {
        // Robot-relative override
        var inputChassisSpeeds = controlSuppliers.getChassisSpeeds(false, _inputs.GyroAngle,
            () -> {
              setAutoAlignEnabled(false);
              _inputs.DrivingRobotRelative = false;
            });

        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(inputChassisSpeeds, _inputs.GyroAngle));
      } else {
        driveRobotRelative(controlSuppliers.getChassisSpeeds(false, _inputs.GyroAngle,
            () -> setAutoAlignEnabled(false)));
      }

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
   * Enables lock-on control to whichever target is in view
   */
  public Command enableReefAutoAlignCommand() {
    var cmd = Commands.run(() -> {
      var frontLimelightInputs = Container.Vision.getLimelightInputs(0);

      // If targeted AprilTag is in validTargets, align to its offset
      if (VisionSubsystem.isReefTag(frontLimelightInputs.ApriltagId)) {
        var targetHeadingOffsetDeg = frontLimelightInputs.RobotSpaceTargetPose.Pose
            .getRotation()
            .toRotation2d()
            // .plus(Rotation2d.fromDegrees(180)) // TODO: Enable this if the target pose facing the robot is flipped
            .getDegrees();

        var adjustedTargetAngle = _inputs.GyroAngle.getDegrees() + targetHeadingOffsetDeg;

        // Set the drivetrain to align to the target heading
        _autoAlign.setSetpoint(Rotation2d.fromDegrees(adjustedTargetAngle));
        // setAutoAlignEnabled(true);
        // _inputs.DriveRobotRelative = true;
      } else {
        // setAutoAlignEnabled(false);
        // _inputs.DriveRobotRelative = false;
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

  /**
   * Will move to a side of the reef
   * @param side
   */
  public Command pathfindToReefPegSide(ReefPegSide side) {
    return this.defer(() -> {
      var desiredPose = getReefPegsPoseFromInput(side, _inputs.EstimatedRobotPose);
      Container.TeleopDashboardSection.getFieldTargetPose().setPose(desiredPose);

      // Create trajectory
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          List.of(_inputs.EstimatedRobotPose, desiredPose),
          new TrajectoryConfig(0.3, 0.165) // 16.5 cm/s max speed & acceleration
      );

      return new SwerveControllerCommand(
          trajectory,
          () -> _inputs.EstimatedRobotPose,
          _swervePackager.Kinematics, // Your kinematics object
          SwerveMap.PathPlannerTranslationPID.createPIDController(0.02), // X controller
          SwerveMap.PathPlannerTranslationPID.createPIDController(0.02), // Y controller
          new ProfiledPIDController(
              SwerveMap.PathPlannerRotationPID.kP,
              SwerveMap.PathPlannerRotationPID.kI,
              SwerveMap.PathPlannerRotationPID.kD,
              new Constraints(SwerveMap.Chassis.MaxAngularSpeedRadians, SwerveMap.Chassis.MaxAngularSpeedRadians / 2)), // Theta controller
          () -> desiredPose.getRotation(),
          _swervePackager::setDesiredModuleStates, // Sends the states to the SwerveModules
          this)
          .andThen(stopAllMotorsCommand())
          .andThen(Commands.print("Reached desired pose " + desiredPose));
    });
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
    return Map.of(
        getName() + "-EnableTargetLockOn",
        enableReefAutoAlignCommand(),
        getName() + "-DisableAutoAlign",
        disableAutoAlignCommand(),
        getName() + "-EnableAutoAlignRotationFeedback",
        enablePathPlannerAutoAlignRotationFeedbackCommand(),
        getName() + "-DisableAutoAlignRotationFeedback",
        disablePathPlannerAutoAlignRotationFeedbackCommand(),
        getName() + "-MoveToLeftPegs",
        pathfindToReefPegSide(ReefPegSide.kLeft),
        getName() + "-MoveToRightPegs",
        pathfindToReefPegSide(ReefPegSide.kRight));
  }
  // #endregion
}
