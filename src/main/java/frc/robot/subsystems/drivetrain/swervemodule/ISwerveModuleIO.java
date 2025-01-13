package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ISwerveModuleIO {

  public SwerveModuleIOInputs getInputs();

  public void setOutputs(SwerveModuleIOOutputs outputs);

  public void setDriveVoltage(double voltage, Rotation2d moduleAngle);

  public void stopMotors();
}
