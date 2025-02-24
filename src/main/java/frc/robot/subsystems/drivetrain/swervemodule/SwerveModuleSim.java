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
import frc.robot.dashboard.DashboardSection;
import frc.robot.subsystems.drivetrain.SwerveMap;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.util.SwerveUtil;

public class SwerveModuleSim implements ISwerveModule {
  private String _name;
  // private SwerveModuleMap _map;
  private DashboardSection _dashboardSection;
  private final String _optimizeModuleKey = "Optimize";

  // Devices
  private DCMotorSim _driveMotorSim;
  private PIDController _drivingPidController;
  private SimpleMotorFeedforward _driveFeedForward;
  private Rotation2d _currentHeading = new Rotation2d();

  public SwerveModuleSim(String name, SwerveModuleMap moduleMap) {
    _name = name;
    // _map = moduleMap;
    _dashboardSection = new DashboardSection("Drivetrain/" + _name);
    _dashboardSection.putBoolean(_optimizeModuleKey, true);

    setupDriveMotor(SwerveMap.DrivePID);
  }

  @Override
  public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
    _driveMotorSim.update(0.020);
    var speedMps = _driveMotorSim.getAngularVelocity().in(Units.RotationsPerSecond)
        * SwerveMap.DriveWheelCircumferenceMeters;

    inputs.ModuleState.angle = _currentHeading;
    inputs.ModuleState.speedMetersPerSecond = speedMps;
    inputs.ModulePosition.angle = _currentHeading;
    inputs.ModulePosition.distanceMeters = _driveMotorSim.getAngularPositionRotations()
        * SwerveMap.DriveWheelCircumferenceMeters;
    Logger.recordOutput("Drive/" + _name + "/DriveMotorMeasuredVoltage", _driveMotorSim.getInputVoltage());
  }

  @Override
  public void setDriveVoltage(double voltage, Rotation2d moduleAngle) {
    _driveMotorSim.setInputVoltage(voltage);
    _currentHeading = moduleAngle;
  }

  @Override
  public void stopMotors() {
    _driveMotorSim.setInputVoltage(0);
    _driveMotorSim.setAngularVelocity(0);
  }

  @Override
  public void setDrivePID(ExtendedPIDConstants drivePID) {
    var currentSetpoint = _drivingPidController.getSetpoint();
    _drivingPidController = drivePID.createPIDController(0.02);
    _drivingPidController.setSetpoint(currentSetpoint);

    _driveFeedForward = new SimpleMotorFeedforward(drivePID.kS, drivePID.kV, drivePID.kA);
  }

  @Override
  public void setSteeringPID(ExtendedPIDConstants steeringPID) {
    // Not implemented
  }

  /**
   * Configures the drive motors
   * 
   * @param pid
   */
  private void setupDriveMotor(ExtendedPIDConstants pid) {
    _driveMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, SwerveMap.DriveGearRatio),
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
    var optimize = _dashboardSection.getBoolean(_optimizeModuleKey, true);
    Logger.recordOutput("Drive/" + _name + "/Optimized", optimize);
    if (optimize) {
      desiredState = SwerveUtil.optimize(desiredState, _currentHeading);
    }

    Logger.recordOutput("Drive/" + _name + "/SteeringMotorOutputSpeed", 0);
    _currentHeading = desiredState.angle;

    // Set the drive motor to the desired speed
    // Calculate target data to voltage data
    var desiredSpeedRotationsPerSecond = (desiredState.speedMetersPerSecond / SwerveMap.DriveWheelCircumferenceMeters)
        * SwerveMap.DriveGearRatio;

    var ff = _driveFeedForward.calculate(desiredSpeedRotationsPerSecond);

    var currentSpeedRotationsPerSecond = _driveMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
    var pid = _drivingPidController.calculate(currentSpeedRotationsPerSecond, desiredSpeedRotationsPerSecond);
    var driveOutput = MathUtil.clamp(ff + pid, -12.0, 12.0);

    Logger.recordOutput("Drive/" + _name + "/DrivePID", pid);
    Logger.recordOutput("Drive/" + _name + "/DriveFF", ff);
    Logger.recordOutput("Drive/" + _name + "/DriveMotorOutputVoltage", driveOutput);
    _driveMotorSim.setInputVoltage(driveOutput);
  }
}
