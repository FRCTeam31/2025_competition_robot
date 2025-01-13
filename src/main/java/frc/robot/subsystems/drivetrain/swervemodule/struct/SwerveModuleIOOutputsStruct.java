package frc.robot.subsystems.drivetrain.swervemodule.struct;

import java.nio.ByteBuffer;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleIOOutputs;

public class SwerveModuleIOOutputsStruct implements Struct<SwerveModuleIOOutputs> {

    @Override
    public Class<SwerveModuleIOOutputs> getTypeClass() {
        return SwerveModuleIOOutputs.class;
    }

    @Override
    public String getTypeName() {
        return SwerveModuleIOOutputs.class.getName();
    }

    @Override
    public int getSize() {
        return SwerveModuleState.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "SwerveModuleState DesiredState";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {SwerveModuleState.struct};
    }

    @Override
    public SwerveModuleIOOutputs unpack(ByteBuffer bb) {
        var state = SwerveModuleState.struct.unpack(bb);
        return new SwerveModuleIOOutputs(state);
    }

    @Override
    public void pack(ByteBuffer bb, SwerveModuleIOOutputs value) {
        SwerveModuleState.struct.pack(bb, value.DesiredState);
    }

}
