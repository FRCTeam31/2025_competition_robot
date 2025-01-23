package frc.robot.subsystems.drivetrain.swervemodule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

@AutoLog
public class SwerveModuleOutputs {
    public SwerveModuleOutputs() {
    }

    public SwerveModuleOutputs(SwerveModuleState desiredState) {
        DesiredState = desiredState;
    }

    public SwerveModuleState DesiredState = new SwerveModuleState();
}
