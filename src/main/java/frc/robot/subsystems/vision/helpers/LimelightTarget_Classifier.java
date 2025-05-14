package frc.robot.subsystems.vision.helpers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightTarget_Classifier implements LoggableInputs, Cloneable {

    @JsonProperty("class")
    public String className;

    @JsonProperty("classID")
    public double classID;

    @JsonProperty("conf")
    public double confidence;

    @JsonProperty("zone")
    public double zone;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double ty_pixels;

    public LimelightTarget_Classifier() {
    }

    @Override
    public void toLog(LogTable table) {
        table.put("ClassName", className);
        table.put("ClassID", classID);
        table.put("Confidence", confidence);
        table.put("Zone", zone);
        table.put("Tx", tx);
        table.put("Tx_pixels", tx_pixels);
        table.put("Ty", ty);
        table.put("Ty_pixels", ty_pixels);
    }

    @Override
    public void fromLog(LogTable table) {
        className = table.get("ClassName", className);
        classID = table.get("ClassID", classID);
        confidence = table.get("Confidence", confidence);
        zone = table.get("Zone", zone);
        tx = table.get("Tx", tx);
        tx_pixels = table.get("Tx_pixels", tx_pixels);
        ty = table.get("Ty", ty);
        ty_pixels = table.get("Ty_pixels", ty_pixels);
    }

    public LimelightTarget_Classifier clone() {
        LimelightTarget_Classifier copy = new LimelightTarget_Classifier();
        copy.className = this.className;
        copy.classID = this.classID;
        copy.confidence = this.confidence;
        copy.zone = this.zone;
        copy.tx = this.tx;
        copy.tx_pixels = this.tx_pixels;
        copy.ty = this.ty;
        copy.ty_pixels = this.ty_pixels;
        return copy;
    }
}
