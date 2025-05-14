package frc.robot.subsystems.vision.helpers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RawDetection implements LoggableInputs, Cloneable {
    public int classId = 0;
    public double txnc = 0;
    public double tync = 0;
    public double ta = 0;
    public double corner0_X = 0;
    public double corner0_Y = 0;
    public double corner1_X = 0;
    public double corner1_Y = 0;
    public double corner2_X = 0;
    public double corner2_Y = 0;
    public double corner3_X = 0;
    public double corner3_Y = 0;

    public RawDetection(int classId, double txnc, double tync, double ta,
            double corner0_X, double corner0_Y,
            double corner1_X, double corner1_Y,
            double corner2_X, double corner2_Y,
            double corner3_X, double corner3_Y) {
        this.classId = classId;
        this.txnc = txnc;
        this.tync = tync;
        this.ta = ta;
        this.corner0_X = corner0_X;
        this.corner0_Y = corner0_Y;
        this.corner1_X = corner1_X;
        this.corner1_Y = corner1_Y;
        this.corner2_X = corner2_X;
        this.corner2_Y = corner2_Y;
        this.corner3_X = corner3_X;
        this.corner3_Y = corner3_Y;
    }

    public RawDetection() {
    }

    @Override
    public void toLog(LogTable table) {
        table.put("ClassId", classId);
        table.put("Txnc", txnc);
        table.put("Tync", tync);
        table.put("Ta", ta);
        table.put("Corner0_X", corner0_X);
        table.put("Corner0_Y", corner0_Y);
        table.put("Corner1_X", corner1_X);
        table.put("Corner1_Y", corner1_Y);
        table.put("Corner2_X", corner2_X);
        table.put("Corner2_Y", corner2_Y);
        table.put("Corner3_X", corner3_X);
        table.put("Corner3_Y", corner3_Y);
    }

    @Override
    public void fromLog(LogTable table) {
        classId = table.get("ClassId", classId);
        txnc = table.get("Txnc", txnc);
        tync = table.get("Tync", tync);
        ta = table.get("Ta", ta);
        corner0_X = table.get("Corner0_X", corner0_X);
        corner0_Y = table.get("Corner0_Y", corner0_Y);
        corner1_X = table.get("Corner1_X", corner1_X);
        corner1_Y = table.get("Corner1_Y", corner1_Y);
        corner2_X = table.get("Corner2_X", corner2_X);
        corner2_Y = table.get("Corner2_Y", corner2_Y);
        corner3_X = table.get("Corner3_X", corner3_X);
        corner3_Y = table.get("Corner3_Y", corner3_Y);
    }

    public RawDetection clone() {
        RawDetection copy = new RawDetection();
        copy.classId = this.classId;
        copy.txnc = this.txnc;
        copy.tync = this.tync;
        copy.ta = this.ta;
        copy.corner0_X = this.corner0_X;
        copy.corner0_Y = this.corner0_Y;
        copy.corner1_X = this.corner1_X;
        copy.corner1_Y = this.corner1_Y;
        copy.corner2_X = this.corner2_X;
        copy.corner2_Y = this.corner2_Y;
        copy.corner3_X = this.corner3_X;
        copy.corner3_Y = this.corner3_Y;
        return copy;
    }
}
