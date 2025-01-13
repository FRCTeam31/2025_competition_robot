package prime.vision;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;

@Logged
public class LimelightPose implements StructSerializable {

  public Pose3d Pose = new Pose3d();
  public double Timestamp = 0.0;
  public double TagCount = 0.0;
  public double TagSpan = 0.0;
  public double AvgTagDistanceMeters = 0.0;
  public double AvgTagArea = 0.0;
  public double[] StdDeviations = new double[3];

  public LimelightPose() {}

  public LimelightPose(double[] poseTagPipelineData, double[] stdDeviations) {
    if (poseTagPipelineData.length < 11 || poseTagPipelineData.length > 11) {
      System.err.println("Bad LL 3D Pose Data!");
      return;
    }

    if (stdDeviations.length < 3 || stdDeviations.length > 3) {
      System.err.println("Bad LL StdDeviations Data!");
      return;
    }

    Pose =
      new Pose3d(
        new Translation3d(poseTagPipelineData[0], poseTagPipelineData[1], poseTagPipelineData[2]),
        new Rotation3d(
          Units.degreesToRadians(poseTagPipelineData[3]),
          Units.degreesToRadians(poseTagPipelineData[4]),
          Units.degreesToRadians(poseTagPipelineData[5])
        )
      );

    var latencyMs = poseTagPipelineData[6];
    Timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
    TagCount = poseTagPipelineData[7];
    TagSpan = poseTagPipelineData[8];
    AvgTagDistanceMeters = poseTagPipelineData[9];
    AvgTagArea = poseTagPipelineData[10];
    StdDeviations = stdDeviations;
  }

  public LimelightPose(Pose3d pose, double[] tagPipelineData, double[] stdDeviations) {
    if (tagPipelineData.length < 5 || tagPipelineData.length > 5) {
      System.err.println("Bad LL 3D Pose Data!");
      return;
    }

    if (stdDeviations.length < 3 || stdDeviations.length > 3) {
      System.err.println("Bad LL StdDeviations Data!");
      return;
    }

    Pose = pose;

    var latencyMs = tagPipelineData[0];
    Timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
    TagCount = tagPipelineData[1];
    TagSpan = tagPipelineData[2];
    AvgTagDistanceMeters = tagPipelineData[3];
    AvgTagArea = tagPipelineData[4];
    StdDeviations = stdDeviations;
  }

  public LimelightPose(Pose3d pose, double[] tagPipelineData) {
    if (tagPipelineData.length < 5 || tagPipelineData.length > 5) {
      System.err.println("Bad LL 3D Pose Data!");
      return;
    }

    Pose = pose;

    var latencyMs = tagPipelineData[0];
    Timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
    TagCount = tagPipelineData[1];
    TagSpan = tagPipelineData[2];
    AvgTagDistanceMeters = tagPipelineData[3];
    AvgTagArea = tagPipelineData[4];
  }

  @NotLogged
  public Matrix<N3, N1> getStdDeviations() {
    return new Matrix<N3, N1>(new SimpleMatrix(StdDeviations));
  }

  /** Struct for serialization. */
  public static final LimelightPoseStruct struct = new LimelightPoseStruct();
}
