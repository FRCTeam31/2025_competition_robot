package org.prime.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightInputs implements LoggableInputs, Cloneable {

    /**
     * Horizontal Offset From Crosshair To Target 
     * (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     */
    public Rotation2d TargetHorizontalOffset = new Rotation2d(Math.PI);

    /**
     * Vertical Offset From Crosshair To Target 
     * (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     */
    public Rotation2d TargetVerticalOffset = new Rotation2d();

    /**
     * Target Area (0% of image to 100% of image)
     */
    public double TargetArea = 0.0;

    /**
     * The pipeline's latency contribution (ms). Add to "cl" to get total latency.
     */
    public int PipelineLatencyMs = 0;

    /**
     * Time between the end of the exposure of the middle row of the sensor to 
     * the beginning of the tracking pipeline.
     */
    public int CapturePipelineLatencyMs = 0;

    /**
     * The total latency of the capture and pipeline processing in milliseconds.
     */
    public int TotalLatencyMs = 0;

    /**
     * ID of the primary in-view AprilTag
     */
    public int ApriltagId = -1;

    /**
     * Returns the number of AprilTags in the image.
     */
    public double TagCount = 0;

    /**
     * Robot transform in field-space.
     */
    public LimelightPose FieldSpaceRobotPose = new LimelightPose();

    /**
     * Robot transform in field-space (alliance driverstation WPILIB origin).
     */
    public LimelightPose RedAllianceOriginFieldSpaceRobotPose = new LimelightPose();

    /**
     * Robot transform in field-space (alliance driverstation WPILIB origin).
     */
    public LimelightPose BlueAllianceOriginFieldSpaceRobotPose = new LimelightPose();

    /**
     * 3D transform of the robot in the coordinate system of the primary in-view AprilTag
     */
    public LimelightPose TargetSpaceRobotPose = new LimelightPose();

    /**
     * 3D transform of the camera in the coordinate system of the primary in-view AprilTag
     */
    public LimelightPose TargetSpaceCameraPose = new LimelightPose();

    /**
     * 3D transform of the camera in the coordinate system of the robot
     */
    public LimelightPose RobotSpaceCameraPose = new LimelightPose();

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate system of the Camera
     */
    public LimelightPose CameraSpaceTargetPose = new LimelightPose();

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate system of the Robot
     */
    public LimelightPose RobotSpaceTargetPose = new LimelightPose();

    @Override
    public void toLog(LogTable table) {
        table.put("TargetHorizontalOffset", TargetHorizontalOffset);
        table.put("TargetVerticalOffset", TargetVerticalOffset);
        table.put("TargetArea", TargetArea);
        table.put("PipelineLatencyMs", PipelineLatencyMs);
        table.put("CapturePipelineLatencyMs", CapturePipelineLatencyMs);
        table.put("TotalLatencyMs", TotalLatencyMs);
        table.put("ApriltagId", ApriltagId);
        table.put("TagCount", TagCount);
        table.put("FieldSpaceRobotPose", FieldSpaceRobotPose);
        table.put("RedAllianceOriginFieldSpaceRobotPose", RedAllianceOriginFieldSpaceRobotPose);
        table.put("BlueAllianceOriginFieldSpaceRobotPose", BlueAllianceOriginFieldSpaceRobotPose);
        table.put("TargetSpaceRobotPose", TargetSpaceRobotPose);
        table.put("TargetSpaceCameraPose", TargetSpaceCameraPose);
        table.put("RobotSpaceCameraPose", RobotSpaceCameraPose);
        table.put("CameraSpaceTargetPose", CameraSpaceTargetPose);
        table.put("RobotSpaceTargetPose", RobotSpaceTargetPose);
    }

    @Override
    public void fromLog(LogTable table) {
        TargetHorizontalOffset = table.get("TargetHorizontalOffset", TargetHorizontalOffset);
        TargetVerticalOffset = table.get("TargetVerticalOffset", TargetVerticalOffset);
        TargetArea = table.get("TargetArea", TargetArea);
        PipelineLatencyMs = table.get("PipelineLatencyMs", PipelineLatencyMs);
        CapturePipelineLatencyMs = table.get("CapturePipelineLatencyMs", CapturePipelineLatencyMs);
        TotalLatencyMs = table.get("TotalLatencyMs", TotalLatencyMs);
        ApriltagId = table.get("ApriltagId", ApriltagId);
        TagCount = table.get("TagCount", TagCount);
        FieldSpaceRobotPose = table.get("FieldSpaceRobotPose", FieldSpaceRobotPose);
        RedAllianceOriginFieldSpaceRobotPose = table.get("RedAllianceOriginFieldSpaceRobotPose",
                RedAllianceOriginFieldSpaceRobotPose);
        BlueAllianceOriginFieldSpaceRobotPose = table.get("BlueAllianceOriginFieldSpaceRobotPose",
                BlueAllianceOriginFieldSpaceRobotPose);
        TargetSpaceRobotPose = table.get("TargetSpaceRobotPose", TargetSpaceRobotPose);
        TargetSpaceCameraPose = table.get("TargetSpaceCameraPose", TargetSpaceCameraPose);
        RobotSpaceCameraPose = table.get("RobotSpaceCameraPose", RobotSpaceCameraPose);
        CameraSpaceTargetPose = table.get("CameraSpaceTargetPose", CameraSpaceTargetPose);
        RobotSpaceTargetPose = table.get("RobotSpaceTargetPose", RobotSpaceTargetPose);
    }

    public LimelightInputs clone() {
        LimelightInputs copy = new LimelightInputs();
        copy.TargetHorizontalOffset = this.TargetHorizontalOffset;
        copy.TargetVerticalOffset = this.TargetVerticalOffset;
        copy.TargetArea = this.TargetArea;
        copy.PipelineLatencyMs = this.PipelineLatencyMs;
        copy.CapturePipelineLatencyMs = this.CapturePipelineLatencyMs;
        copy.TotalLatencyMs = this.TotalLatencyMs;
        copy.ApriltagId = this.ApriltagId;
        copy.TagCount = this.TagCount;
        copy.FieldSpaceRobotPose = this.FieldSpaceRobotPose;
        copy.RedAllianceOriginFieldSpaceRobotPose = this.RedAllianceOriginFieldSpaceRobotPose;
        copy.BlueAllianceOriginFieldSpaceRobotPose = this.BlueAllianceOriginFieldSpaceRobotPose;
        copy.TargetSpaceRobotPose = this.TargetSpaceRobotPose;
        copy.TargetSpaceCameraPose = this.TargetSpaceCameraPose;
        copy.RobotSpaceCameraPose = this.RobotSpaceCameraPose;
        copy.CameraSpaceTargetPose = this.CameraSpaceTargetPose;
        copy.RobotSpaceTargetPose = this.RobotSpaceTargetPose;
        return copy;
    }
}
