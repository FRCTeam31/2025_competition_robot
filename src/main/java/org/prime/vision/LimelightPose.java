package org.prime.vision;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class LimelightPose implements LoggableInputs, Cloneable {

  public Pose3d Pose = new Pose3d();
  public double Timestamp = 0.0;
  public double TagCount = 0.0;
  public double TagSpan = 0.0;
  public double AvgTagDistanceMeters = 0.0;
  public double AvgTagArea = 0.0;
  public double[] StdDeviations = new double[3];

  public LimelightPose() {
  }

  public LimelightPose(double[] poseTagPipelineData) {
    if (poseTagPipelineData.length >= 6) {
      Pose = new Pose3d(
          new Translation3d(poseTagPipelineData[0], poseTagPipelineData[1], poseTagPipelineData[2]),
          new Rotation3d(
              Units.degreesToRadians(poseTagPipelineData[3]),
              Units.degreesToRadians(poseTagPipelineData[4]),
              Units.degreesToRadians(poseTagPipelineData[5])));

      StdDeviations = calculateTrust(1, 0.25);
    }

    if (poseTagPipelineData.length >= 7) {
      var latencyMs = poseTagPipelineData[6];
      Timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
    } else {
      Timestamp = Timer.getFPGATimestamp();
    }

    if (poseTagPipelineData.length >= 11) {
      TagCount = poseTagPipelineData[7];
      TagSpan = poseTagPipelineData[8];
      AvgTagDistanceMeters = poseTagPipelineData[9];
      AvgTagArea = poseTagPipelineData[10];
      StdDeviations = calculateTrust(TagCount, AvgTagArea);
    }
  }

  private double[] calculateTrust(double tagCount, double tagArea) {
    double trustLevel;

    if (tagCount <= 0) {
      trustLevel = 20;
    } else {
      trustLevel = tagCount >= 2.0 ? 2 : 20;
    }

    return VecBuilder.fill(trustLevel, trustLevel, 9999999).getData();
  }

  // public LimelightPose(Pose3d pose, double[] tagPipelineData, double[] stdDeviations) {
  //   if (tagPipelineData.length < 5 || tagPipelineData.length > 5) {
  //     System.err.println("Bad LL 3D Pose Data!");
  //     return;
  //   }

  //   if (stdDeviations.length < 3 || stdDeviations.length > 3) {
  //     System.err.println("Bad LL StdDeviations Data!");
  //     return;
  //   }

  //   Pose = pose;

  //   var latencyMs = tagPipelineData[0];
  //   Timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
  //   TagCount = tagPipelineData[1];
  //   TagSpan = tagPipelineData[2];
  //   AvgTagDistanceMeters = tagPipelineData[3];
  //   AvgTagArea = tagPipelineData[4];
  //   StdDeviations = stdDeviations;
  // }

  // public LimelightPose(Pose3d pose, double[] tagPipelineData) {
  //   if (tagPipelineData.length < 5 || tagPipelineData.length > 5) {
  //     System.err.println("Bad LL 3D Pose Data!");
  //     return;
  //   }

  //   Pose = pose;

  //   var latencyMs = tagPipelineData[0];
  //   Timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
  //   TagCount = tagPipelineData[1];
  //   TagSpan = tagPipelineData[2];
  //   AvgTagDistanceMeters = tagPipelineData[3];
  //   AvgTagArea = tagPipelineData[4];
  // }

  public Matrix<N3, N1> getStdDeviations() {
    return new Matrix<N3, N1>(new SimpleMatrix(StdDeviations));
  }

  @Override
  public void toLog(LogTable table) {
    table.put("Pose", Pose);
    table.put("Timestamp", Timestamp);
    table.put("TagCount", TagCount);
    table.put("TagSpan", TagSpan);
    table.put("AvgTagDistanceMeters", AvgTagDistanceMeters);
    table.put("AvgTagArea", AvgTagArea);
    table.put("StdDeviations", StdDeviations);
  }

  @Override
  public void fromLog(LogTable table) {
    Pose = table.get("Pose", Pose);
    Timestamp = table.get("Timestamp", Timestamp);
    TagCount = table.get("TagCount", TagCount);
    TagSpan = table.get("TagSpan", TagSpan);
    AvgTagDistanceMeters = table.get("AvgTagDistanceMeters", AvgTagDistanceMeters);
    AvgTagArea = table.get("AvgTagArea", AvgTagArea);
    StdDeviations = table.get("StdDeviations", StdDeviations);
  }

  public LimelightPose clone() {
    LimelightPose copy = new LimelightPose();
    copy.Pose = this.Pose;
    copy.Timestamp = this.Timestamp;
    copy.TagCount = this.TagCount;
    copy.TagSpan = this.TagSpan;
    copy.AvgTagDistanceMeters = this.AvgTagDistanceMeters;
    copy.AvgTagArea = this.AvgTagArea;
    copy.StdDeviations = this.StdDeviations.clone();
    return copy;
  }
}
