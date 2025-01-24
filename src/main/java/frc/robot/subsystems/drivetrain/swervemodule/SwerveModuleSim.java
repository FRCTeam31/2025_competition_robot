package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drivetrain.DriveMap;

import org.littletonrobotics.junction.Logger;
import org.prime.control.PrimePIDConstants;

public class SwerveModuleSim implements ISwerveModule {
  private String _name;

  // Devices
  private DCMotorSim _driveMotorSim;
  private PIDController _drivingPidController;
  private SimpleMotorFeedforward _driveFeedForward;
  private Rotation2d _steerAngle = new Rotation2d();

  public SwerveModuleSim(String name, SwerveModuleMap moduleMap) {
    _name = name;
    setupDriveMotor(DriveMap.DrivePID);
  }

  @Override
  public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
    _driveMotorSim.update(0.020);
    var speedMps = _driveMotorSim.getAngularVelocity().in(Units.RotationsPerSecond)
        * DriveMap.DriveWheelCircumferenceMeters;

    inputs.ModuleState.angle = _steerAngle;
    inputs.ModuleState.speedMetersPerSecond = speedMps;
    inputs.ModulePosition.angle = _steerAngle;
    inputs.ModulePosition.distanceMeters = _driveMotorSim.getAngularPositionRotations()
        * DriveMap.DriveWheelCircumferenceMeters;
    Logger.recordOutput("Drive/" + _name + "/DriveMotorMeasuredVoltage", _driveMotorSim.getInputVoltage());
  }

  @Override
  public void setDriveVoltage(double voltage, Rotation2d moduleAngle) {
    _driveMotorSim.setInputVoltage(voltage);
    _steerAngle = moduleAngle;
  }

  @Override
  public void stopMotors() {
    _driveMotorSim.setInputVoltage(0);
    _driveMotorSim.setAngularVelocity(0);
  }

  /**
   * Configures the drive motors
   * 
   * @param pid
   */
  private void setupDriveMotor(PrimePIDConstants pid) {
    _driveMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, DriveMap.DriveGearRatio),
        DCMotor.getNeoVortex(1));

    _drivingPidController = pid.createPIDController(0.02);
    _driveFeedForward = new SimpleMotorFeedforward(pid.kS, pid.kV, pid.kA);
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

    Logger.recordOutput("Drive/" + _name + "/SteeringMotorOutputSpeed", 0);
    _steerAngle = desiredState.angle;

    // Set the drive motor to the desired speed
    // Calculate target data to voltage data
    var desiredSpeedRotationsPerSecond = (desiredState.speedMetersPerSecond / DriveMap.DriveWheelCircumferenceMeters)
        * DriveMap.DriveGearRatio;

    var ff = _driveFeedForward.calculate(desiredSpeedRotationsPerSecond);

    var currentSpeedRotationsPerSecond = _driveMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
    var pid = _drivingPidController.calculate(currentSpeedRotationsPerSecond, desiredSpeedRotationsPerSecond);
    var driveOutput = MathUtil.clamp(ff + pid, -12.0, 12.0);

    Logger.recordOutput("Drive/" + _name + "/DrivePID", pid);
    Logger.recordOutput("Drive/" + _name + "/DriveFF", ff);
    Logger.recordOutput("Drive/" + _name + "/DriveMotorOutputVoltage", driveOutput);
    _driveMotorSim.setInputVoltage(driveOutput);
  }

  /**
   * Optimizes the module angle & drive inversion to ensure the module takes the
   * shortest path to drive at the desired angle
   * 
   * @param desiredState
   */
  private SwerveModuleState optimize(SwerveModuleState desiredState) {
    var delta = desiredState.angle.minus(_steerAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(-desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return desiredState;
    }
  }
}
