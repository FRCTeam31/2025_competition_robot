package frc.robot.subsystems.drivetrain.swervemodule.struct;

import java.nio.ByteBuffer;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleIOInputs;

public class SwerveModuleIOInputsStruct implements Struct<SwerveModuleIOInputs> {

  @Override
  public Class<SwerveModuleIOInputs> getTypeClass() {
    return SwerveModuleIOInputs.class;
  }

  @Override
  public String getTypeName() {
    return SwerveModuleIOInputs.class.getName();
  }

  @Override
  public int getSize() {
    return SwerveModulePosition.struct.getSize() + SwerveModuleState.struct.getSize() + kSizeDouble;
  }

  @Override
  public String getSchema() {
    return "SwerveModulePosition ModulePosition;SwerveModuleState ModuleState;double DriveMotorVoltage";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] { SwerveModulePosition.struct, SwerveModuleState.struct };
  }

  @Override
  public SwerveModuleIOInputs unpack(ByteBuffer bb) {
    var position = SwerveModulePosition.struct.unpack(bb);
    var state = SwerveModuleState.struct.unpack(bb);
    var inputs = new SwerveModuleIOInputs(position, state);
    inputs.DriveMotorVoltage = bb.getDouble();

    return inputs;
  }

  @Override
  public void pack(ByteBuffer bb, SwerveModuleIOInputs value) {
    SwerveModulePosition.struct.pack(bb, value.ModulePosition);
    SwerveModuleState.struct.pack(bb, value.ModuleState);
    bb.putDouble(value.DriveMotorVoltage);
  }

}
