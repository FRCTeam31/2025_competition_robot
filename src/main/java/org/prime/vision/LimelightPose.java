package org.prime.vision;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.MathUtil;
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

      StdDeviations = calculateTrust(0.5);
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
      StdDeviations = calculateTrust(AvgTagDistanceMeters, TagCount);
    }
  }

  /**
   * Calculate the trust level based on the area of the tag. Interpolates between some known trust values.
   * @param tagArea
   * @return
   */
  private double[] calculateTrust(double tagArea) {
    var trustLevel = LimelightUtil.StdDeviationAreaTreeMap.get(tagArea);
    trustLevel = MathUtil.clamp(trustLevel, 2, 15);

    return VecBuilder.fill(trustLevel, trustLevel, 9999999).getData();
  }

  /**
   * Calculate the standard deviations for the pose based on the average tag distance and the number of tags.
   * Shamelessly taken from 2383's 2025 code: https://github.com/Ninjineers-2383/REEFSCAPE/blob/f6e2f0a69e47640b84fc4e9a6b0997a6f02d401f/src/main/java/frc/robot/subsystems/vision/Vision.java#L119C9-L122C69
   */
  private double[] calculateTrust(double avgTagDistance, double tagCount) {
    var stdDevFactor = Math.pow(avgTagDistance, 2.0) / tagCount;
    var linearStdDev = 0.02 * stdDevFactor; // multiply by a baseline in meters
    var angularStdDev = 0.06 * stdDevFactor; // multiply by a baseline in radians

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev).getData();
  }

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
