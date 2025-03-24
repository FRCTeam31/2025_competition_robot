package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import frc.robot.dashboard.DashboardSection;
import frc.robot.subsystems.swerve.SwerveMap;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.util.SwerveUtil;

public class SwerveModuleReal implements ISwerveModule {
  private String _name;
  private SwerveModuleMap _map;
  private DashboardSection _dashboardSection;
  private final String _optimizeModuleKey = "Optimize";

  // Devices
  private SparkFlex _steeringMotor;
  private PIDController _steeringPidController;
  private SparkFlex _driveMotor;
  private PIDController _drivingPidController;
  private SimpleMotorFeedforward _driveFeedForward;
  private CANcoder _encoder;

  public SwerveModuleReal(String name, SwerveModuleMap moduleMap) {
    _name = name;
    _map = moduleMap;
    _dashboardSection = new DashboardSection("Drive/" + _name);
    _dashboardSection.putBoolean(_optimizeModuleKey, true);

    setupSteeringMotor(SwerveMap.SteeringPID);
    setupDriveMotor(SwerveMap.DrivePID);
    setupCanCoder();
  }

  /**
   * Configures the steering motor and PID controller
   */
  private void setupSteeringMotor(ExtendedPIDConstants pid) {
    _steeringMotor = new SparkFlex(_map.SteeringMotorCanId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(_map.SteerInverted); // CCW inversion
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(30, 20);

    _steeringMotor.clearFaults();
    _steeringMotor.configure(config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Create a PID controller to calculate steering motor output
    _steeringPidController = pid.createPIDController(0.02);
    _steeringPidController.enableContinuousInput(0, 1); // 0 to 1 rotation
  }

  @Override
  public void setSteeringPID(ExtendedPIDConstants steeringPID) {
    _steeringPidController.setP(steeringPID.kP);
    _steeringPidController.setI(steeringPID.kI);
    _steeringPidController.setD(steeringPID.kD);
    System.out.println("Reset Steering PID " + _name);
  }

  /**
   * Configures the drive motors
   * 
   * @param pid
   */
  private void setupDriveMotor(ExtendedPIDConstants pid) {
    _driveMotor = new SparkFlex(_map.DriveMotorCanId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(SwerveMap.DriveStallCurrentLimit, SwerveMap.DriveFreeCurrentLimit);
    config.openLoopRampRate(_map.DriveMotorRampRate);
    config.inverted(_map.DriveInverted);
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.reverseLimitSwitchEnabled(false);
    config.voltageCompensation(12);

    // Apply the configuration
    _driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create a PID controller to calculate driving motor output
    _drivingPidController = pid.createPIDController(0.02);
    _driveFeedForward = new SimpleMotorFeedforward(pid.kS, pid.kV, pid.kA);
  }

  @Override
  public void setDrivePID(ExtendedPIDConstants drivePID) {
    _drivingPidController.setP(drivePID.kP);
    _drivingPidController.setI(drivePID.kI);
    _drivingPidController.setD(drivePID.kD);
    _driveFeedForward = new SimpleMotorFeedforward(drivePID.kS, drivePID.kV, drivePID.kA);
    System.out.println("Reset Drive PID " + _name);
  }

  /**
   * Configures the CANCoder
   */
  private void setupCanCoder() {
    _encoder = new CANcoder(_map.CANCoderCanId);
    _encoder.clearStickyFaults();
    _encoder.getConfigurator().apply(new CANcoderConfiguration());

    // AbsoluteSensorRangeValue
    _encoder.getConfigurator()
        .apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(1)
                .withMagnetOffset(-_map.CanCoderStartingOffset)));
  }

  @Override
  public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
    var rotation = getCurrentHeading();
    var speedMps = getCurrentVelocity().in(MetersPerSecond);
    var distanceMeters = getModuleDistance();

    inputs.ModuleState.angle = rotation;
    inputs.ModuleState.speedMetersPerSecond = speedMps;
    inputs.ModulePosition.angle = rotation;
    inputs.ModulePosition.distanceMeters = distanceMeters.magnitude();
    inputs.DriveMotorVoltage = _driveMotor.getAppliedOutput() * _driveMotor.getBusVoltage();
    Logger.recordOutput("Swerve/Modules/" + _name + "/DriveMotorMeasuredVoltage",
        _driveMotor.getAppliedOutput() * _driveMotor.getBusVoltage());
  }

  @Override
  public void setDriveVoltage(double voltage, Rotation2d moduleAngle) {

    _driveMotor.setVoltage(voltage);

    setModuleAngle(moduleAngle);
  }

  @Override
  public void stopMotors() {
    _driveMotor.stopMotor();
    _steeringMotor.stopMotor();
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The optimized state of the module that we'd like to be at in this period
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state
    var optimize = _dashboardSection.getBoolean(_optimizeModuleKey, true);
    Logger.recordOutput("Swerve/Modules/" + _name + "/Optimized", optimize);
    if (optimize) {
      desiredState = SwerveUtil.optimize(desiredState, getCurrentHeading());
    }

    // Set the drive motor to the desired speed
    setDriveSpeed(desiredState.speedMetersPerSecond);

    // Set the steering motor to the desired angle
    setModuleAngle(desiredState.angle);
  }

  private void setDriveSpeed(double desiredSpeedMetersPerSecond) {
    // Convert the speed to rotations per second by dividing by the wheel
    // circumference and gear ratio

    var desiredSpeedRotationsPerSecond = (desiredSpeedMetersPerSecond / SwerveMap.DriveWheelCircumferenceMeters)
        * SwerveMap.DriveGearRatio;

    var currentSpeedMetersPerSecond = getCurrentVelocity().in(MetersPerSecond);
    var currentSpeedRotationsPerSecond = (currentSpeedMetersPerSecond / SwerveMap.DriveWheelCircumferenceMeters)
        * SwerveMap.DriveGearRatio;

    var pid = _drivingPidController.calculate(currentSpeedRotationsPerSecond, desiredSpeedRotationsPerSecond);
    var ff = _driveFeedForward.calculate(desiredSpeedRotationsPerSecond);
    var driveOutput = MathUtil.clamp(pid + ff, -12, 12);

    Logger.recordOutput("Swerve/Modules/" + _name + "/DrivePID", pid);
    Logger.recordOutput("Swerve/Modules/" + _name + "/DriveFF", ff);
    Logger.recordOutput("Swerve/Modules/" + _name + "/DriveMotorOutputVoltage", driveOutput);
    _driveMotor.setVoltage(driveOutput);
  }

  private void setModuleAngle(Rotation2d angle) {
    // Normalize to 0 to 1
    var setpoint = angle.getRotations() % 1;
    if (setpoint < 0)
      setpoint += 1;

    // Calculate the new output using the PID controller
    var newOutput = _steeringPidController.calculate(getCurrentHeading().getRotations(), setpoint);
    var steerSpeed = MathUtil.clamp(newOutput, -1, 1);

    // Set the steering motor's speed to the calculated output
    Logger.recordOutput("Swerve/Modules/" + _name + "/SteeringMotorOutputSpeed", steerSpeed);
    _steeringMotor.set(steerSpeed);
  }

  /**
   * Gets the current heading of the module
   */
  private Rotation2d getCurrentHeading() {
    return Rotation2d.fromRotations(_encoder.getPosition().getValueAsDouble());
  }

  /**
   * Gets the current velocity of the module
   */
  private MutLinearVelocity getCurrentVelocity() {
    var speedMps = ((_driveMotor.getEncoder().getVelocity() / 60) / SwerveMap.DriveGearRatio)
        * SwerveMap.DriveWheelCircumferenceMeters;

    return Units.MetersPerSecond.mutable(speedMps);
  }

  /**
   * Gets the distance the module has traveled
   */
  private MutDistance getModuleDistance() {
    var distMeters = _driveMotor.getEncoder().getPosition()
        * (SwerveMap.DriveWheelCircumferenceMeters / SwerveMap.DriveGearRatio);

    return Meters.mutable(distMeters);
  }
}
