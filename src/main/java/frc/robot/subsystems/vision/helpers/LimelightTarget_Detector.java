package frc.robot.subsystems.vision.helpers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightTarget_Detector implements LoggableInputs, Cloneable {

    @JsonProperty("class")
    public String className;

    @JsonProperty("classID")
    public double classID;

    @JsonProperty("conf")
    public double confidence;

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double ty_pixels;

    public LimelightTarget_Detector() {
    }

    @Override
    public void toLog(LogTable table) {
        table.put("ClassName", className);
        table.put("ClassID", classID);
        table.put("Confidence", confidence);
        table.put("Ta", ta);
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
        ta = table.get("Ta", ta);
        tx = table.get("Tx", tx);
        tx_pixels = table.get("Tx_pixels", tx_pixels);
        ty = table.get("Ty", ty);
        ty_pixels = table.get("Ty_pixels", ty_pixels);
    }

    public LimelightTarget_Detector clone() {
        LimelightTarget_Detector copy = new LimelightTarget_Detector();
        copy.className = this.className;
        copy.classID = this.classID;
        copy.confidence = this.confidence;
        copy.ta = this.ta;
        copy.tx = this.tx;
        copy.tx_pixels = this.tx_pixels;
        copy.ty = this.ty;
        copy.ty_pixels = this.ty_pixels;
        return copy;
    }
}
