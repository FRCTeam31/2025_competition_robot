package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ISwerveModule {

  public void updateInputs(SwerveModuleInputsAutoLogged inputs);

  public void setOutputs(SwerveModuleOutputsAutoLogged outputs);

  public void setDriveVoltage(double voltage, Rotation2d moduleAngle);

  public void stopMotors();
}
