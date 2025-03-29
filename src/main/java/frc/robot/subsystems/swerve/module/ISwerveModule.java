package frc.robot.subsystems.swerve.module;

import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {

  /**
   * Updates the module's inputs
   * @param inputs The inputs object to write updates to
   */
  public void updateInputs(SwerveModuleInputsAutoLogged inputs);

  /**
   * Sets the desired output states of the module, the angle and speed
   */
  public void setDesiredState(SwerveModuleState desiredState);

  /**
   * Sets a voltage to the drive motor and locks the module at a particular heading
   */
  public void setDriveVoltage(double voltage, Rotation2d moduleAngle);

  /**
   * Stops drive and steering motors
   */
  public void stopMotors();

  /**
   * Reconfigures the drive PID and FF
   */
  public void setDrivePID(ExtendedPIDConstants drivePID);

  /**
   * Reconfigures the steering PID
   */
  public void setSteeringPID(ExtendedPIDConstants steeringPID);
}
