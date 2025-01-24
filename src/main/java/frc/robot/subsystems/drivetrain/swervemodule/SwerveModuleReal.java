package frc.robot.subsystems.drivetrain.swervemodule;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drivetrain.DriveMap;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;
import org.prime.control.PrimePIDConstants;

public class SwerveModuleReal implements ISwerveModule {
  private String _name;
  private SwerveModuleMap _map;

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

    setupSteeringMotor(DriveMap.SteeringPID);
    setupDriveMotor(DriveMap.DrivePID);
    setupCanCoder();
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
    Logger.recordOutput("Drive/" + _name + "/DriveMotorMeasuredVoltage",
        _driveMotor.getAppliedOutput() * _driveMotor.getBusVoltage());
  }

  @Override
  public void setDriveVoltage(double voltage, Rotation2d moduleAngle) {
    var speed = voltage / _driveMotor.getBusVoltage();
    _driveMotor.set(speed);

    setModuleAngle(moduleAngle);
  }

  @Override
  public void stopMotors() {
    _driveMotor.stopMotor();
    _steeringMotor.stopMotor();
  }

  /**
   * Configures the steering motor and PID controller
   */
  private void setupSteeringMotor(PrimePIDConstants pid) {
    _steeringMotor = new SparkFlex(_map.SteeringMotorCanId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(_map.SteerInverted); // CCW inversion
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(60, 50);

    _steeringMotor.clearFaults();
    _steeringMotor.configure(config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Create a PID controller to calculate steering motor output
    _steeringPidController = pid.createPIDController(0.02);
    _steeringPidController.enableContinuousInput(0, 1); // 0 to 1 rotation
    _steeringPidController.setTolerance((1 / 360.0) * 0.05);
  }

  /**
   * Configures the drive motors
   * 
   * @param pid
   */
  private void setupDriveMotor(PrimePIDConstants pid) {
    _driveMotor = new SparkFlex(_map.DriveMotorCanId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(60, 50);
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
    _drivingPidController.setTolerance(0.001);
    _driveFeedForward = new SimpleMotorFeedforward(pid.kS, pid.kV, pid.kA);
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
            .withMagnetSensor(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5)
                .withMagnetOffset(-_map.CanCoderStartingOffset)));
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The optimized state of the module that we'd like to be at
   *                     in this
   *                     period
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state
    desiredState = optimize(desiredState);

    // Set the drive motor to the desired speed
    setDriveSpeed(desiredState.speedMetersPerSecond);

    // Set the steering motor to the desired angle
    setModuleAngle(desiredState.angle);
  }

  private void setDriveSpeed(double desiredSpeedMetersPerSecond) {
    // Convert the speed to rotations per second by dividing by the wheel
    // circumference and gear ratio

    var desiredSpeedRotationsPerSecond = (desiredSpeedMetersPerSecond / DriveMap.DriveWheelCircumferenceMeters)
        * DriveMap.DriveGearRatio;

    var currentSpeedMetersPerSecond = getCurrentVelocity().in(MetersPerSecond);
    var currentSpeedRotationsPerSecond = (currentSpeedMetersPerSecond / DriveMap.DriveWheelCircumferenceMeters)
        * DriveMap.DriveGearRatio;

    var pid = _drivingPidController.calculate(currentSpeedRotationsPerSecond, desiredSpeedRotationsPerSecond);
    var ff = _driveFeedForward.calculate(desiredSpeedRotationsPerSecond);
    var driveOutput = MathUtil.clamp(pid + ff, -12, 12);

    Logger.recordOutput("Drive/" + _name + "/DrivePID", pid);
    Logger.recordOutput("Drive/" + _name + "/DriveFF", ff);
    Logger.recordOutput("Drive/" + _name + "/DriveMotorOutputVoltage", driveOutput);
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
    Logger.recordOutput("Drive/" + _name + "/SteeringMotorOutputSpeed", steerSpeed);
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
  private LinearVelocity getCurrentVelocity() {
    var speedMps = ((_driveMotor.getEncoder().getVelocity() / 60) / DriveMap.DriveGearRatio)
        * DriveMap.DriveWheelCircumferenceMeters;

    return Units.MetersPerSecond.of(speedMps);
  }

  private Distance getModuleDistance() {
    var distMeters = (_driveMotor.getEncoder().getPosition() / DriveMap.DriveGearRatio)
        * DriveMap.DriveWheelCircumferenceMeters;

    return Meters.of(distMeters);
  }

  /**
   * Optimizes the module angle & drive inversion to ensure the module takes the
   * shortest path to drive at the desired angle
   * 
   * @param desiredState
   */
  private SwerveModuleState optimize(SwerveModuleState desiredState) {
    Rotation2d currentAngle = getCurrentHeading();
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(-desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return desiredState;
    }
  }
}
