package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.subsystems.drivetrain.swervemodule.struct.SwerveModuleIOOutputsStruct;

@Logged
public class SwerveModuleIOOutputs implements StructSerializable {

    /** Struct for serialization. */
    public static final SwerveModuleIOOutputsStruct struct = new SwerveModuleIOOutputsStruct();

    public SwerveModuleIOOutputs() { }

    public SwerveModuleIOOutputs(SwerveModuleState desiredState) {
        DesiredState = desiredState;
    }

    public SwerveModuleState DesiredState = new SwerveModuleState();
}
