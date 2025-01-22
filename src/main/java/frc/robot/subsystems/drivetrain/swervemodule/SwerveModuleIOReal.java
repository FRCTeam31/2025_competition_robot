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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.maps.*;

import prime.control.PrimePIDConstants;

public class SwerveModuleIOReal implements ISwerveModuleIO {

  private SwerveModuleMap m_map;
  private SwerveModuleIOInputs m_inputs = new SwerveModuleIOInputs();

  // Devices
  private SparkFlex m_SteeringMotor;
  private PIDController m_steeringPidController;
  private PIDController m_drivingPidController;
  private SimpleMotorFeedforward driveFeedForward;
  private SparkFlex m_driveMotor;
  private CANcoder m_encoder;

  public SwerveModuleIOReal(SwerveModuleMap moduleMap) {
    m_map = moduleMap;

    setupSteeringMotor(DriveMap.SteeringPID);
    setupDriveMotor(DriveMap.DrivePID);
    setupCanCoder();

  }

  @Override
  public SwerveModuleIOInputs getInputs() {
    var rotation = Rotation2d.fromRotations(m_encoder.getPosition().getValueAsDouble());
    var speedMps = ((m_driveMotor.getEncoder().getVelocity() / 60) / DriveMap.DriveGearRatio)
        * DriveMap.DriveWheelCircumferenceMeters;
    var distanceMeters = Units.Rotations.of(m_driveMotor.getEncoder().getPosition())
        .times(DriveMap.DriveWheelCircumferenceMeters * DriveMap.DriveGearRatio);

    m_inputs.ModuleState.angle = rotation;
    m_inputs.ModuleState.speedMetersPerSecond = speedMps;
    m_inputs.ModulePosition.angle = rotation;
    m_inputs.ModulePosition.distanceMeters = distanceMeters.magnitude();
    m_inputs.DriveMotorVoltage = m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();

    return m_inputs;
  }

  @Override
  public void setOutputs(SwerveModuleIOOutputs outputs) {
    if (!outputs.VoltageDrive) {
      setDesiredState(outputs.DesiredState);
    }
  }

  @Override
  public void setDriveVoltage(double voltage, Rotation2d moduleAngle) {
    m_driveMotor.setVoltage(voltage);

    setModuleAngle(moduleAngle);
  }

  @Override
  public void stopMotors() {
    m_driveMotor.stopMotor();
    m_SteeringMotor.stopMotor();
  }

  /**
   * Configures the steering motor and PID controller
   */
  private void setupSteeringMotor(PrimePIDConstants pid) {
    m_SteeringMotor = new SparkFlex(m_map.SteeringMotorCanId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(m_map.SteerInverted); // CCW inversion
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(60, 50);

    m_SteeringMotor.clearFaults();
    m_SteeringMotor.configure(config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Create a PID controller to calculate steering motor output
    m_steeringPidController = pid.createPIDController(0.02);
    m_steeringPidController.enableContinuousInput(0, 1); // 0 to 1 rotation
    m_steeringPidController.setTolerance((1 / 360.0) * 0.1); // 2 degrees in units of rotations
  }

  /**
   * Configures the drive motors
   * 
   * @param pid
   */
  private void setupDriveMotor(PrimePIDConstants pid) {
    m_driveMotor = new SparkFlex(m_map.DriveMotorCanId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(60, 50);
    config.openLoopRampRate(m_map.DriveMotorRampRate);
    config.inverted(m_map.DriveInverted);
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.reverseLimitSwitchEnabled(false);
    config.voltageCompensation(12);

    // Apply the configuration
    m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create a PID controller to calculate driving motor output
    m_drivingPidController = pid.createPIDController(0.02);
    m_drivingPidController.setTolerance(0.001);
    driveFeedForward = new SimpleMotorFeedforward(pid.kS, pid.kV, pid.kA);
  }

  /**
   * Configures the CANCoder
   */
  private void setupCanCoder() {
    m_encoder = new CANcoder(m_map.CANCoderCanId);
    m_encoder.clearStickyFaults();
    m_encoder.getConfigurator().apply(new CANcoderConfiguration());

    // AbsoluteSensorRangeValue
    m_encoder.getConfigurator()
        .apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(1)
                .withMagnetOffset(-m_map.CanCoderStartingOffset)));
  }

  /**
   * Sets the desired state of the module
   *
   * @param desiredState The optimized state of the module that we'd like to be at
   *                     in this
   *                     period
   */
  private void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state
    desiredState = optimize(desiredState);

    // Set the drive motor to the desired speed
    setDriveSpeed(desiredState.speedMetersPerSecond);

    // Set the steering motor to the desired angle
    setModuleAngle(desiredState.angle);
  }

  private void setDriveSpeed(double speedMetersPerSecond) {
    // Convert the speed to rotations per second by dividing by the wheel
    // circumference and gear ratio

    var desiredSpeedRotationsPerSecond = (speedMetersPerSecond / DriveMap.DriveWheelCircumferenceMeters)
        * DriveMap.DriveGearRatio;

    var currentSpeedRotationsPerSecond = (m_inputs.ModuleState.speedMetersPerSecond
        / DriveMap.DriveWheelCircumferenceMeters) * DriveMap.DriveGearRatio;

    var pid = m_drivingPidController.calculate(currentSpeedRotationsPerSecond, desiredSpeedRotationsPerSecond);
    var ff = driveFeedForward.calculate(desiredSpeedRotationsPerSecond);
    var driveOutput = MathUtil.clamp(pid + ff, -12, 12);

    SmartDashboard.putNumber("currentSpeedRPS", currentSpeedRotationsPerSecond);
    SmartDashboard.putNumber("desiredSpeedRPS", desiredSpeedRotationsPerSecond);
    SmartDashboard.putNumber("desiredSpeedMPS", speedMetersPerSecond);
    SmartDashboard.putNumber("currentSpeedMPS", m_inputs.ModuleState.speedMetersPerSecond);

    m_driveMotor.setVoltage(driveOutput);
  }

  private void setModuleAngle(Rotation2d angle) {
    // Normalize to 0 to 1
    var setpoint = angle.getRotations() % 1;
    if (setpoint < 0)
      setpoint += 1;

    // Calculate the new output using the PID controller
    var newOutput = m_steeringPidController.calculate(m_inputs.ModuleState.angle.getRotations(), setpoint);

    // Set the steering motor's speed to the calculated output
    m_SteeringMotor.set(MathUtil.clamp(newOutput, -1, 1));
  }

  /**
   * Optimizes the module angle & drive inversion to ensure the module takes the
   * shortest path to drive at the desired angle
   * 
   * @param desiredState
   */
  private SwerveModuleState optimize(SwerveModuleState desiredState) {
    Rotation2d currentAngle = m_inputs.ModulePosition.angle;
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(-desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return desiredState;
    }
  }
}
