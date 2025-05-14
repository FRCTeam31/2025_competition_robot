package frc.robot.subsystems.vision.helpers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class LimelightTarget_Fiducial implements LoggableInputs, Cloneable {

    @JsonProperty("fID")
    public double fiducialID;

    @JsonProperty("fam")
    public String fiducialFamily;

    @JsonProperty("t6c_ts")
    public double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    public double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    public double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    public double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    public double[] targetPose_RobotSpace;

    public Pose3d getCameraPose_TargetSpace() {
        return LimelightHelpers.toPose3D(cameraPose_TargetSpace);
    }

    public Pose3d getRobotPose_FieldSpace() {
        return LimelightHelpers.toPose3D(robotPose_FieldSpace);
    }

    public Pose3d getRobotPose_TargetSpace() {
        return LimelightHelpers.toPose3D(robotPose_TargetSpace);
    }

    public Pose3d getTargetPose_CameraSpace() {
        return LimelightHelpers.toPose3D(targetPose_CameraSpace);
    }

    public Pose3d getTargetPose_RobotSpace() {
        return LimelightHelpers.toPose3D(targetPose_RobotSpace);
    }

    public Pose2d getCameraPose_TargetSpace2D() {
        return LimelightHelpers.toPose2D(cameraPose_TargetSpace);
    }

    public Pose2d getRobotPose_FieldSpace2D() {
        return LimelightHelpers.toPose2D(robotPose_FieldSpace);
    }

    public Pose2d getRobotPose_TargetSpace2D() {
        return LimelightHelpers.toPose2D(robotPose_TargetSpace);
    }

    public Pose2d getTargetPose_CameraSpace2D() {
        return LimelightHelpers.toPose2D(targetPose_CameraSpace);
    }

    public Pose2d getTargetPose_RobotSpace2D() {
        return LimelightHelpers.toPose2D(targetPose_RobotSpace);
    }

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

    @JsonProperty("ts")
    public double ts;

    public LimelightTarget_Fiducial() {
        cameraPose_TargetSpace = new double[6];
        robotPose_FieldSpace = new double[6];
        robotPose_TargetSpace = new double[6];
        targetPose_CameraSpace = new double[6];
        targetPose_RobotSpace = new double[6];
    }

    @Override
    public void toLog(LogTable table) {
        table.put("FiducialID", fiducialID);
        table.put("FiducialFamily", fiducialFamily);
        table.put("CameraPose_TargetSpace", cameraPose_TargetSpace);
        table.put("RobotPose_FieldSpace", robotPose_FieldSpace);
        table.put("RobotPose_TargetSpace", robotPose_TargetSpace);
        table.put("TargetPose_CameraSpace", targetPose_CameraSpace);
        table.put("TargetPose_RobotSpace", targetPose_RobotSpace);
        table.put("Ta", ta);
        table.put("Tx", tx);
        table.put("Tx_pixels", tx_pixels);
        table.put("Ty", ty);
        table.put("Ty_pixels", ty_pixels);
        table.put("Ts", ts);
    }

    @Override
    public void fromLog(LogTable table) {
        fiducialID = table.get("FiducialID", fiducialID);
        fiducialFamily = table.get("FiducialFamily", fiducialFamily);
        cameraPose_TargetSpace = table.get("CameraPose_TargetSpace", cameraPose_TargetSpace);
        robotPose_FieldSpace = table.get("RobotPose_FieldSpace", robotPose_FieldSpace);
        robotPose_TargetSpace = table.get("RobotPose_TargetSpace", robotPose_TargetSpace);
        targetPose_CameraSpace = table.get("TargetPose_CameraSpace", targetPose_CameraSpace);
        targetPose_RobotSpace = table.get("TargetPose_RobotSpace", targetPose_RobotSpace);
        ta = table.get("Ta", ta);
        tx = table.get("Tx", tx);
        tx_pixels = table.get("Tx_pixels", tx_pixels);
        ty = table.get("Ty", ty);
        ty_pixels = table.get("Ty_pixels", ty_pixels);
        ts = table.get("Ts", ts);
    }

    public LimelightTarget_Fiducial clone() {
        LimelightTarget_Fiducial copy = new LimelightTarget_Fiducial();
        copy.fiducialID = this.fiducialID;
        copy.fiducialFamily = this.fiducialFamily;
        copy.cameraPose_TargetSpace = this.cameraPose_TargetSpace.clone();
        copy.robotPose_FieldSpace = this.robotPose_FieldSpace.clone();
        copy.robotPose_TargetSpace = this.robotPose_TargetSpace.clone();
        copy.targetPose_CameraSpace = this.targetPose_CameraSpace.clone();
        copy.targetPose_RobotSpace = this.targetPose_RobotSpace.clone();
        copy.ta = this.ta;
        copy.tx = this.tx;
        copy.tx_pixels = this.tx_pixels;
        copy.ty = this.ty;
        copy.ty_pixels = this.ty_pixels;
        copy.ts = this.ts;
        return copy;
    }
}
