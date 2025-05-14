package frc.robot.subsystems.vision.helpers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RawFiducial implements LoggableInputs, Cloneable {
    public int id = 0;
    public double txnc = 0;
    public double tync = 0;
    public double ta = 0;
    public double distToCamera = 0;
    public double distToRobot = 0;
    public double ambiguity = 0;

    public RawFiducial(int id,
            double txnc,
            double tync,
            double ta,
            double distToCamera,
            double distToRobot,
            double ambiguity) {
        this.id = id;
        this.txnc = txnc;
        this.tync = tync;
        this.ta = ta;
        this.distToCamera = distToCamera;
        this.distToRobot = distToRobot;
        this.ambiguity = ambiguity;
    }

    public RawFiducial() {
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Id", id);
        table.put("Txnc", txnc);
        table.put("Tync", tync);
        table.put("Ta", ta);
        table.put("DistToCamera", distToCamera);
        table.put("DistToRobot", distToRobot);
        table.put("Ambiguity", ambiguity);
    }

    @Override
    public void fromLog(LogTable table) {
        id = table.get("Id", id);
        txnc = table.get("Txnc", txnc);
        tync = table.get("Tync", tync);
        ta = table.get("Ta", ta);
        distToCamera = table.get("DistToCamera", distToCamera);
        distToRobot = table.get("DistToRobot", distToRobot);
        ambiguity = table.get("Ambiguity", ambiguity);
    }

    public RawFiducial clone() {
        RawFiducial copy = new RawFiducial();
        copy.id = this.id;
        copy.txnc = this.txnc;
        copy.tync = this.tync;
        copy.ta = this.ta;
        copy.distToCamera = this.distToCamera;
        copy.distToRobot = this.distToRobot;
        copy.ambiguity = this.ambiguity;
        return copy;
    }
}
