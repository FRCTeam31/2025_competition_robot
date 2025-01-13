package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.subsystems.drivetrain.swervemodule.struct.SwerveModuleIOInputsStruct;

@Logged
public class SwerveModuleIOInputs implements StructSerializable {
  /** SwerveModuleIOInputs struct for serialization. */
  public static final SwerveModuleIOInputsStruct struct = new SwerveModuleIOInputsStruct();

  public SwerveModuleIOInputs() {
  }

  public SwerveModuleIOInputs(SwerveModulePosition modulePosition, SwerveModuleState moduleState) {
    ModulePosition = modulePosition;
    ModuleState = moduleState;
  }

  public SwerveModulePosition ModulePosition = new SwerveModulePosition();
  public SwerveModuleState ModuleState = new SwerveModuleState();
  public double DriveMotorVoltage = 0.0;
}
