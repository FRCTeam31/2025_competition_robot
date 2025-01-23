package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.DriverDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionSubsystem;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.prime.control.SwerveControlSuppliers;
import org.prime.vision.LimelightInputs;

public class DrivetrainSubsystem extends SubsystemBase {

  public enum DrivetrainControlMode {
    kRobotRelative, kFieldRelative, kPathFollowing,
  }

  private Runnable _clearForegroundPatternFunc;
  private Consumer<LEDPattern> _setForegroundPatternFunc;

  // Shuffleboard Drivetrain tab configuration
  private ShuffleboardTab d_drivetrainTab = Shuffleboard.getTab("Drivetrain");
  public GenericEntry d_snapToEnabledEntry = d_drivetrainTab.add("SnapTo Enabled", false)
      .withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).withSize(2, 2).getEntry();
  private GenericEntry d_snapAngle = d_drivetrainTab.add("SnapTo Angle", 0)
      .withWidget(BuiltInWidgets.kGyro).withPosition(2, 0).withSize(4, 5)
      .withProperties(
          Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0))
      .getEntry();

  // IO
  private SwerveController _swerveController;
  private SwerveControllerInputsAutoLogged _inputs = new SwerveControllerInputsAutoLogged();

  // SnapAngle
  private boolean _snapAngleEnabled = false;
  private Rotation2d _snapSetpoint = Rotation2d.fromDegrees(0);
  private PIDController _snapAnglePIDController;

  // Vision, Kinematics, odometry
  private Supplier<LimelightInputs[]> _limelightInputsSupplier;
  public boolean EstimatePoseUsingFrontCamera = true;
  public boolean EstimatePoseUsingRearCamera = true;
  public boolean WithinPoseEstimationVelocity = true;

  private LEDPattern _snapOnTargetPattern = LEDPattern
      .solid(Color.kGreen)
      .blink(Units.Seconds.of(0.1));
  private LEDPattern _snapOffTargetPattern = LEDPattern
      .steps(Map.of(0.0, Color.kRed, 0.25, Color.kBlack))
      .scrollAtRelativeSpeed(Units.Hertz.of(2));

  SysIdRoutine _driveSysIdRoutine;

  /**
   * Creates a new Drivetrain.
   */
  public DrivetrainSubsystem(boolean isReal, Runnable clearForegroundPatternFunc,
      Consumer<LEDPattern> setForegroundPatternFunc,
      Supplier<LimelightInputs[]> limelightInputsSupplier) {
    setName("Drivetrain");

    _swerveController = new SwerveController(isReal);
    _swerveController.updateInputs(_inputs);

    // Configure snap-to PID
    _snapAnglePIDController = DriveMap.SnapToPID.createPIDController(0.02);
    _snapAnglePIDController.enableContinuousInput(-Math.PI, Math.PI);
    _snapAnglePIDController.setTolerance(Math.PI / 180d);

    // Store references to LED pattern control funcs
    _clearForegroundPatternFunc = clearForegroundPatternFunc;
    _setForegroundPatternFunc = setForegroundPatternFunc;

    // Store reference to limelight inputs supplier
    _limelightInputsSupplier = limelightInputsSupplier;

    configurePathPlanner();

    // Create a new SysId routine for characterizing the drive.
    // TODO: Remove when no longer needed
    _driveSysIdRoutine = new SysIdRoutine(
        // Ramp up at 1 volt per second for quasistatic tests, step at 2 volts in
        // dynamic tests, run for 13 seconds.
        new SysIdRoutine.Config(Units.Volts.of(0.5).per(Units.Second), Units.Volts.of(2),
            Units.Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            _swerveController::setDriveVoltages,
            // Tell SysId how to record a frame of data for each motor on the mechanism
            // being characterized.
            _swerveController::logSysIdDrive,
            // Tell SysId to make generated commands require this subsystem, suffix test
            // state in WPILog with this subsystem's name
            this));
  }

  private void configurePathPlanner() {
    // Load the RobotConfig from the GUI settings, or use the default if an
    // exception occurs
    RobotConfig config = DriveMap.PathPlannerRobotConfiguration;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Set up PP to feed current path poses to the dashboard's field widget
    PathPlannerLogging.setLogCurrentPoseCallback(pose -> DriverDashboard.FieldWidget.setRobotPose(pose));
    PathPlannerLogging
        .setLogTargetPoseCallback(pose -> DriverDashboard.FieldWidget.getObject("target pose").setPose(pose));
    PathPlannerLogging.setLogActivePathCallback(poses -> DriverDashboard.FieldWidget.getObject("path").setPoses(poses));

    // Configure PathPlanner holonomic control
    AutoBuilder.configure(() -> _inputs.EstimatedRobotPose, _swerveController::setEstimatorPose,
        () -> _inputs.RobotRelativeChassisSpeeds,
        (speeds, feedForwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            DriveMap.PathPlannerTranslationPID.toPIDConstants(),
            DriveMap.PathPlannerRotationPID.toPIDConstants()),
        config, () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          var alliance = DriverStation.getAlliance();

          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        }, this);

    // Override PathPlanner's rotation feedback
    // PPHolonomicDriveController.overrideRotationFeedback(this::getSnapAngleCorrection);

  }

  // #region Control methods

  public void resetGyro() {
    _swerveController.resetGyro();
  }

  /**
   * Enabled/disables snap-to control
   */
  private void setSnapToEnabled(boolean enabled) {
    _snapAngleEnabled = enabled;
    if (!enabled)
      _clearForegroundPatternFunc.run();
  }

  /**
   * Sets the snap-to gyro setpoint, converting from degrees to radians
   * 
   * @param angle The angle to snap to in degrees
   */
  private void setSnapToSetpoint(double angle) {
    var setpointModulated = MathUtil.angleModulus(Rotation2d.fromDegrees(angle).getRadians());
    _snapSetpoint = Rotation2d.fromRadians(setpointModulated);

    setSnapToEnabled(true);
  }

  /**
   * Drives robot-relative using a ChassisSpeeds
   * 
   * @param robotRelativeChassisSpeeds The desired speeds of the robot
   */
  private void driveRobotRelative(ChassisSpeeds robotRelativeChassisSpeeds) {
    // If snap-to is enabled, calculate and override the input rotational speed to
    // reach the setpoint
    robotRelativeChassisSpeeds.omegaRadiansPerSecond = _snapAngleEnabled
        ? getSnapAngleCorrection(_inputs.GyroAngle)
        : robotRelativeChassisSpeeds.omegaRadiansPerSecond;

    // Correct drift by taking the input speeds and converting them to a desired
    // per-period speed. This is known as "discretizing"
    robotRelativeChassisSpeeds = ChassisSpeeds.discretize(robotRelativeChassisSpeeds, 0.02);

    // Calculate the module states from the chassis speeds
    var swerveModuleStates = _swerveController.Kinematics.toSwerveModuleStates(robotRelativeChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveMap.Chassis.MaxSpeedMetersPerSecond);

    // Set the desired states for each module
    _swerveController.setDesiredModuleStates(swerveModuleStates);
  }

  /**
   * Drives field-relative using a ChassisSpeeds
   * 
   * @param desiredChassisSpeeds The desired speeds of the robot
   */
  private void driveFieldRelative(ChassisSpeeds desiredChassisSpeeds) {
    // Convert the field-relative speeds to robot-relative
    var robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredChassisSpeeds.vxMetersPerSecond,
        desiredChassisSpeeds.vyMetersPerSecond, desiredChassisSpeeds.omegaRadiansPerSecond, _inputs.GyroAngle);

    driveRobotRelative(robotRelativeSpeeds);
  }

  /**
   * Calculates the snap angle correction using the current gyro angle and the PID controller
   * 
   * @return The rotational speed correction 
   */
  private double getSnapAngleCorrection() {
    return getSnapAngleCorrection(_inputs.GyroAngle);
  }

  /**
   * Calculates the snap angle correction using the PID controller
   * 
   * @param currentAngle The current angle of the robot
   * @return The rotational speed correction 
   */
  private double getSnapAngleCorrection(Rotation2d currentAngle) {
    var currentRotationRadians = MathUtil.angleModulus(currentAngle.getRadians());
    var correction = _snapAnglePIDController.calculate(currentRotationRadians, _snapSetpoint.getRadians());

    return MathUtil.clamp(correction, -DriveMap.Chassis.MaxAngularSpeedRadians,
        DriveMap.Chassis.MaxAngularSpeedRadians);
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

    var limelightInputs = _limelightInputsSupplier.get();

    if (DriverDashboard.FrontPoseEstimationSwitch.getBoolean(false))
      evaluatePoseEstimation(limelightInputs[0]);

    if (DriverDashboard.RearPoseEstimationSwitch.getBoolean(false))
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

    _swerveController.addPoseEstimatorVisionMeasurement(
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
    _swerveController.updateInputs(_inputs);

    processVisionEstimations();

    // Update LEDs
    if (_snapAngleEnabled) {
      _setForegroundPatternFunc.accept(_snapAnglePIDController.atSetpoint()
          ? _snapOnTargetPattern
          : _snapOffTargetPattern);
    }

    // Update shuffleboard
    DriverDashboard.HeadingGyro.setDouble(_inputs.GyroAngle.getDegrees());
    DriverDashboard.FieldWidget.setRobotPose(_inputs.EstimatedRobotPose);
    d_snapToEnabledEntry.setBoolean(_snapAngleEnabled);
    d_snapAngle.setDouble(_snapSetpoint.getRadians());
  }

  // #region Commands

  /**
   * Creates a command that drives the robot using input controls
   * 
   * @param controlSuppliers Controller input suppliers
   */
  public Command driveFieldRelativeCommand(SwerveControlSuppliers controlSuppliers) {
    return this.run(() -> {
      // If the driver is trying to rotate the robot, disable snap-to control
      if (Math.abs(controlSuppliers.Z.getAsDouble()) > 0.2) {
        setSnapToEnabled(false);
        _clearForegroundPatternFunc.run();
      }

      // Convert inputs to MPS
      var inputXMPS = controlSuppliers.X.getAsDouble() * DriveMap.Chassis.MaxSpeedMetersPerSecond;
      var inputYMPS = -controlSuppliers.Y.getAsDouble() * DriveMap.Chassis.MaxSpeedMetersPerSecond;
      var inputRotationRadiansPS = -controlSuppliers.Z.getAsDouble() * DriveMap.Chassis.MaxAngularSpeedRadians;

      // Build chassis speeds
      var invert = Robot.onRedAlliance() ? -1 : 1;

      // Drive the robot with the driver-relative inputs, converted to field-relative
      // based on which side we're on
      var vxSpeed = (inputYMPS * invert); // Driver Y axis is field X axis
      var vySpeed = (inputXMPS * invert); // Driver X axis is field Y axis
      var fieldRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxSpeed, vySpeed,
          inputRotationRadiansPS, _inputs.GyroAngle);

      driveFieldRelative(fieldRelativeChassisSpeeds);
    });
  }

  /**
   * Creates a command that drives the robot using input controls
   * 
   * @param controlSuppliers Controller input suppliers
   */
  public Command driveRobotRelativeCommand(SwerveControlSuppliers controlSuppliers) {
    return this.run(() -> {
      // If the driver is trying to rotate the robot, disable snap-to control
      if (Math.abs(controlSuppliers.Z.getAsDouble()) > 0.2) {
        setSnapToEnabled(false);
        _clearForegroundPatternFunc.run();
      }

      // Convert inputs to MPS
      var inputXMPS = controlSuppliers.X.getAsDouble() * DriveMap.Chassis.MaxSpeedMetersPerSecond;
      var inputYMPS = -controlSuppliers.Y.getAsDouble() * DriveMap.Chassis.MaxSpeedMetersPerSecond;
      var inputRotationRadiansPS = -controlSuppliers.Z.getAsDouble() * DriveMap.Chassis.MaxAngularSpeedRadians;

      // Build chassis speeds
      var invert = Robot.onRedAlliance() ? -1 : 1;

      // Drive the robot with the driver-relative inputs, converted to field-relative
      // based on which side we're on
      var vxSpeed = (inputYMPS * invert);
      var vySpeed = (inputXMPS * invert);
      var robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxSpeed, vySpeed,
          inputRotationRadiansPS, _inputs.GyroAngle);

      driveRobotRelative(robotChassisSpeeds);
    });
  }

  /**
   * Command for resetting the gyro
   */
  public Command resetGyroCommand() {
    return Commands.runOnce(() -> _swerveController.resetGyro());
  }

  /**
   * Enables snap-to control and sets an angle setpoint
   * 
   * @param angle
   */
  public Command setSnapToSetpointCommand(double angle) {
    return Commands.runOnce(() -> setSnapToSetpoint(Robot.onBlueAlliance() ? angle + 180 : angle));
  }

  /**
   * Disables snap-to control
   */
  public Command disableSnapToCommand() {
    return Commands.runOnce(() -> setSnapToEnabled(false));
  }

  /**
   * Enables lock-on control
   */
  public Command enableLockOnCommand() {
    return Commands.run(() -> {
      var rearLimelightInputs = _limelightInputsSupplier.get()[1];

      // If targeted AprilTag is in validTargets, snap to its offset
      if (VisionSubsystem.isAprilTagIdASpeakerCenterTarget(rearLimelightInputs.ApriltagId)) {
        // Calculate the target heading
        var horizontalOffsetDeg = rearLimelightInputs.TargetHorizontalOffset.getDegrees();
        var robotHeadingDeg = _inputs.GyroAngle.getDegrees();
        var targetHeadingDeg = robotHeadingDeg - horizontalOffsetDeg;

        // Set the drivetrain to snap to the target heading
        setSnapToSetpoint(targetHeadingDeg);
      } else {
        setSnapToEnabled(false);
      }
    });
  }

  /**
   * Enables SnapAngle control in PathPlanner routines
   * @return
   */
  public Command enablePathPlannerSnapRotationFeedbackCommand() {
    return Commands.run(() -> {
      PPHolonomicDriveController.overrideRotationFeedback(this::getSnapAngleCorrection);
    });
  }

  /**
   * Disables SnapAngle control in PathPlanner routines
   * @return
   */
  public Command disablePathPlannerSnapRotationFeedbackCommand() {
    return Commands.run(() -> {
      PPHolonomicDriveController.clearRotationFeedbackOverride();
    });
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
        "DisableSnapAngle",
        disableSnapToCommand(),
        "EnableSnapAngleRotationFeedback",
        enablePathPlannerSnapRotationFeedbackCommand(),
        "DisableSnapAngleRotationFeedback",
        disablePathPlannerSnapRotationFeedbackCommand());
  }
  // #endregion
}
