package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {

  public void updateInputs(SwerveModuleInputsAutoLogged inputs);

  public void setDesiredState(SwerveModuleState desiredState);

  public void setDriveVoltage(double voltage, Rotation2d moduleAngle);

  public void stopMotors();
}
