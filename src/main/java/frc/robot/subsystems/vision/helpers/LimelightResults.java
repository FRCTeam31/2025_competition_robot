package frc.robot.subsystems.vision.helpers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class LimelightResults implements LoggableInputs, Cloneable {

    public String error;

    @JsonProperty("pID")
    public double pipelineID;

    @JsonProperty("tl")
    public double latency_pipeline;

    @JsonProperty("cl")
    public double latency_capture;

    public double latency_jsonParse;

    @JsonProperty("ts")
    public double timestamp_LIMELIGHT_publish;

    @JsonProperty("ts_rio")
    public double timestamp_RIOFPGA_capture;

    @JsonProperty("v")
    @JsonFormat(shape = Shape.NUMBER)
    public boolean valid;

    @JsonProperty("botpose")
    public double[] botpose;

    @JsonProperty("botpose_wpired")
    public double[] botpose_wpired;

    @JsonProperty("botpose_wpiblue")
    public double[] botpose_wpiblue;

    @JsonProperty("botpose_tagcount")
    public double botpose_tagcount;

    @JsonProperty("botpose_span")
    public double botpose_span;

    @JsonProperty("botpose_avgdist")
    public double botpose_avgdist;

    @JsonProperty("botpose_avgarea")
    public double botpose_avgarea;

    @JsonProperty("t6c_rs")
    public double[] camerapose_robotspace;

    public Pose3d getBotPose3d() {
        return LimelightHelpers.toPose3D(botpose);
    }

    public Pose3d getBotPose3d_wpiRed() {
        return LimelightHelpers.toPose3D(botpose_wpired);
    }

    public Pose3d getBotPose3d_wpiBlue() {
        return LimelightHelpers.toPose3D(botpose_wpiblue);
    }

    public Pose2d getBotPose2d() {
        return LimelightHelpers.toPose2D(botpose);
    }

    public Pose2d getBotPose2d_wpiRed() {
        return LimelightHelpers.toPose2D(botpose_wpired);
    }

    public Pose2d getBotPose2d_wpiBlue() {
        return LimelightHelpers.toPose2D(botpose_wpiblue);
    }

    @JsonProperty("Retro")
    public LimelightTarget_Retro[] targets_Retro;

    @JsonProperty("Fiducial")
    public LimelightTarget_Fiducial[] targets_Fiducials;

    @JsonProperty("Classifier")
    public LimelightTarget_Classifier[] targets_Classifier;

    @JsonProperty("Detector")
    public LimelightTarget_Detector[] targets_Detector;

    @JsonProperty("Barcode")
    public LimelightTarget_Barcode[] targets_Barcode;

    public LimelightResults() {
        botpose = new double[6];
        botpose_wpired = new double[6];
        botpose_wpiblue = new double[6];
        camerapose_robotspace = new double[6];
        targets_Retro = new LimelightTarget_Retro[0];
        targets_Fiducials = new LimelightTarget_Fiducial[0];
        targets_Classifier = new LimelightTarget_Classifier[0];
        targets_Detector = new LimelightTarget_Detector[0];
        targets_Barcode = new LimelightTarget_Barcode[0];
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Error", error);
        table.put("PipelineID", pipelineID);
        table.put("Latency_pipeline", latency_pipeline);
        table.put("Latency_capture", latency_capture);
        table.put("Latency_jsonParse", latency_jsonParse);
        table.put("Timestamp_LIMELIGHT_publish", timestamp_LIMELIGHT_publish);
        table.put("Timestamp_RIOFPGA_capture", timestamp_RIOFPGA_capture);
        table.put("Valid", valid);
        table.put("Botpose", botpose);
        table.put("Botpose_wpired", botpose_wpired);
        table.put("Botpose_wpiblue", botpose_wpiblue);
        table.put("Botpose_tagcount", botpose_tagcount);
        table.put("Botpose_span", botpose_span);
        table.put("Botpose_avgdist", botpose_avgdist);
        table.put("Botpose_avgarea", botpose_avgarea);
        table.put("Camerapose_robotspace", camerapose_robotspace);
        // table.put("Targets_Retro", targets_Retro);
        // table.put("Targets_Fiducials", targets_Fiducials);
        // table.put("Targets_Classifier", targets_Classifier);
        // table.put("Targets_Detector", targets_Detector);
        // table.put("Targets_Barcode", targets_Barcode);
    }

    @Override
    public void fromLog(LogTable table) {
        error = table.get("Error", error);
        pipelineID = table.get("PipelineID", pipelineID);
        latency_pipeline = table.get("Latency_pipeline", latency_pipeline);
        latency_capture = table.get("Latency_capture", latency_capture);
        latency_jsonParse = table.get("Latency_jsonParse", latency_jsonParse);
        timestamp_LIMELIGHT_publish = table.get("Timestamp_LIMELIGHT_publish", timestamp_LIMELIGHT_publish);
        timestamp_RIOFPGA_capture = table.get("Timestamp_RIOFPGA_capture", timestamp_RIOFPGA_capture);
        valid = table.get("Valid", valid);
        botpose = table.get("Botpose", botpose);
        botpose_wpired = table.get("Botpose_wpired", botpose_wpired);
        botpose_wpiblue = table.get("Botpose_wpiblue", botpose_wpiblue);
        botpose_tagcount = table.get("Botpose_tagcount", botpose_tagcount);
        botpose_span = table.get("Botpose_span", botpose_span);
        botpose_avgdist = table.get("Botpose_avgdist", botpose_avgdist);
        botpose_avgarea = table.get("Botpose_avgarea", botpose_avgarea);
        camerapose_robotspace = table.get("Camerapose_robotspace", camerapose_robotspace);
        // targets_Retro = table.get("Targets_Retro", targets_Retro);
        // targets_Fiducials = table.get("Targets_Fiducials", targets_Fiducials);
        // targets_Classifier = table.get("Targets_Classifier", targets_Classifier);
        // targets_Detector = table.get("Targets_Detector", targets_Detector);
        // targets_Barcode = table.get("Targets_Barcode", targets_Barcode);
    }

    public LimelightResults clone() {
        LimelightResults copy = new LimelightResults();
        copy.error = this.error;
        copy.pipelineID = this.pipelineID;
        copy.latency_pipeline = this.latency_pipeline;
        copy.latency_capture = this.latency_capture;
        copy.latency_jsonParse = this.latency_jsonParse;
        copy.timestamp_LIMELIGHT_publish = this.timestamp_LIMELIGHT_publish;
        copy.timestamp_RIOFPGA_capture = this.timestamp_RIOFPGA_capture;
        copy.valid = this.valid;
        copy.botpose = this.botpose.clone();
        copy.botpose_wpired = this.botpose_wpired.clone();
        copy.botpose_wpiblue = this.botpose_wpiblue.clone();
        copy.botpose_tagcount = this.botpose_tagcount;
        copy.botpose_span = this.botpose_span;
        copy.botpose_avgdist = this.botpose_avgdist;
        copy.botpose_avgarea = this.botpose_avgarea;
        copy.camerapose_robotspace = this.camerapose_robotspace.clone();
        copy.targets_Retro = this.targets_Retro.clone();
        copy.targets_Fiducials = this.targets_Fiducials.clone();
        copy.targets_Classifier = this.targets_Classifier.clone();
        copy.targets_Detector = this.targets_Detector.clone();
        copy.targets_Barcode = this.targets_Barcode.clone();
        return copy;
    }
}
