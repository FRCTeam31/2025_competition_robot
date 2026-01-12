package frc.robot.subsystems.vision.helpers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LimelightTarget_Barcode implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
    }

    @Override
    public void fromLog(LogTable table) {
    }

    public LimelightTarget_Barcode clone() {
        LimelightTarget_Barcode copy = new LimelightTarget_Barcode();
        return copy;
    }
}