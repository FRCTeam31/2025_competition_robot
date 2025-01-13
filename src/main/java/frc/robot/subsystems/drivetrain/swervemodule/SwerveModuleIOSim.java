package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.maps.DriveMap;
import frc.robot.maps.SwerveModuleMap;
import prime.control.PrimePIDConstants;

public class SwerveModuleIOSim implements ISwerveModuleIO {

  private SwerveModuleIOInputs m_inputs = new SwerveModuleIOInputs();

  // Devices
  private DCMotorSim m_driveMotorSim;
  private PIDController m_driveFeedback;
  private SimpleMotorFeedforward m_driveFeedforward;
  private Rotation2d m_steerAngle = new Rotation2d();

  public SwerveModuleIOSim(SwerveModuleMap moduleMap) {
    setupDriveMotor(DriveMap.DrivePID);
  }

  @Override
  public SwerveModuleIOInputs getInputs() {
    m_driveMotorSim.update(0.020);
    var speedMps = m_driveMotorSim.getAngularVelocity().in(Units.RotationsPerSecond)
        * DriveMap.DriveWheelCircumferenceMeters;

    m_inputs.ModuleState.angle = m_steerAngle;
    m_inputs.ModuleState.speedMetersPerSecond = speedMps;
    m_inputs.ModulePosition.angle = m_steerAngle;
    m_inputs.ModulePosition.distanceMeters = m_driveMotorSim.getAngularPositionRotations()
        * DriveMap.DriveWheelCircumferenceMeters;

    return m_inputs;
  }

  @Override
  public void setOutputs(SwerveModuleIOOutputs outputs) {
    setDesiredState(outputs.DesiredState);
  }

  @Override
  public void setDriveVoltage(double voltage, Rotation2d moduleAngle) {
    m_driveMotorSim.setInputVoltage(voltage);
    m_steerAngle = moduleAngle;
  }

  @Override
  public void stopMotors() {
    m_driveMotorSim.setInputVoltage(0);
    m_driveMotorSim.setAngularVelocity(0);
  }

  /**
   * Configures the drive motors
   * 
   * @param pid
   */
  private void setupDriveMotor(PrimePIDConstants pid) {
    m_driveMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNeoVortex(1),
            0.001,
            DriveMap.DriveGearRatio),
        DCMotor.getNeoVortex(1));

    m_driveFeedback = new PIDController(0.1, 0, 0);
    m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.085);
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The optimized state of the module that we'd like to be at
   *                     in this
   *                     period
   */
  private void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state
    desiredState = optimize(desiredState);

    // Set the drive motor to the desired speed
    // Calculate target data to voltage data
    var velocityRadPerSec = desiredState.speedMetersPerSecond / (DriveMap.DriveWheelDiameterMeters / 2);
    var driveAppliedVolts = m_driveFeedforward.calculate(Units.RadiansPerSecond.of(velocityRadPerSec)).magnitude()
        + m_driveFeedback.calculate(m_driveMotorSim.getAngularVelocityRadPerSec(), velocityRadPerSec);
    driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);

    m_driveMotorSim.setInputVoltage(driveAppliedVolts);

    m_steerAngle = desiredState.angle;
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
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return desiredState;
    }
  }
}
