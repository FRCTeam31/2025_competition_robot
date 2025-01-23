package frc.robot.subsystems.drivetrain.swervemodule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@AutoLog
public class SwerveModuleInputs {
  public SwerveModuleInputs() {
  }

  public SwerveModuleInputs(SwerveModulePosition modulePosition, SwerveModuleState moduleState) {
    ModulePosition = modulePosition;
    ModuleState = moduleState;
  }

  public SwerveModulePosition ModulePosition = new SwerveModulePosition();
  public SwerveModuleState ModuleState = new SwerveModuleState();
  public double DriveMotorVoltage = 0.0;
}
