package prime.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;

public class LimelightInputsStruct implements Struct<LimelightInputs> {
    @Override
    public Class<LimelightInputs> getTypeClass() {
        return LimelightInputs.class;
    }

    @Override
    public String getTypeName() {
        return "LimelightInputs";
    }

    @Override
    public int getSize() {
        // Calculate the size based on the fields in LimelightInputs
        int size = Rotation2d.struct.getSize() * 2; // For TargetHorizontalOffset and TargetVerticalOffset
        size += kSizeDouble; // For TargetArea
        size += kSizeInt32 * 4; // For PipelineLatencyMs, CapturePipelineLatencyMs, TotalLatencyMs, and ApriltagId
        size += kSizeDouble; // For TagCount
        size += LimelightPose.struct.getSize() * 8; // For all poses

        return size;
    }

    @Override
    public String getSchema() {
        return "Rotation2d targetHorizontalOffset;"+
        "Rotation2d targetVerticalOffset;"+
        "double targetArea;"+
        "int32 pipelineLatencyMs;"+
        "int32 capturePipelineLatencyMs;"+
        "int32 totalLatencyMs;"+
        "int32 apriltagId;"+
        "double tagCount;"+
        "LimelightPose fieldSpaceRobotPose;"+
        "LimelightPose redAllianceOriginFieldSpaceRobotPose;"+
        "LimelightPose blueAllianceOriginFieldSpaceRobotPose;"+
        "LimelightPose targetSpaceRobotPose;"+
        "LimelightPose targetSpaceCameraPose;"+
        "LimelightPose robotSpaceCameraPose;"+
        "LimelightPose cameraSpaceTargetPose;"+
        "LimelightPose robotSpaceTargetPose";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] { 
            Rotation2d.struct, 
            LimelightPose.struct, 
        };
    }

    @Override
    public LimelightInputs unpack(ByteBuffer bb) {
        LimelightInputs inputs = new LimelightInputs();

        inputs.TargetHorizontalOffset = Rotation2d.struct.unpack(bb);
        inputs.TargetVerticalOffset = Rotation2d.struct.unpack(bb);
        inputs.TargetArea = bb.getDouble();
        inputs.PipelineLatencyMs = bb.getInt();
        inputs.CapturePipelineLatencyMs = bb.getInt();
        inputs.TotalLatencyMs = bb.getInt();
        inputs.ApriltagId = bb.getInt();
        inputs.TagCount = bb.getDouble();
        inputs.FieldSpaceRobotPose = LimelightPose.struct.unpack(bb);
        inputs.RedAllianceOriginFieldSpaceRobotPose = LimelightPose.struct.unpack(bb);
        inputs.BlueAllianceOriginFieldSpaceRobotPose = LimelightPose.struct.unpack(bb);
        inputs.TargetSpaceRobotPose = LimelightPose.struct.unpack(bb);
        inputs.TargetSpaceCameraPose = LimelightPose.struct.unpack(bb);
        inputs.RobotSpaceCameraPose = LimelightPose.struct.unpack(bb);
        inputs.CameraSpaceTargetPose = LimelightPose.struct.unpack(bb);
        inputs.RobotSpaceTargetPose = LimelightPose.struct.unpack(bb);

        return inputs;
    }

    @Override
    public void pack(ByteBuffer bb, LimelightInputs value) {
        Rotation2d.struct.pack(bb, value.TargetHorizontalOffset);
        Rotation2d.struct.pack(bb, value.TargetVerticalOffset);
        bb.putDouble(value.TargetArea);
        bb.putInt(value.PipelineLatencyMs);
        bb.putInt(value.CapturePipelineLatencyMs);
        bb.putInt(value.TotalLatencyMs);
        bb.putInt(value.ApriltagId);
        bb.putDouble(value.TagCount);
        LimelightPose.struct.pack(bb, value.FieldSpaceRobotPose);
        LimelightPose.struct.pack(bb, value.RedAllianceOriginFieldSpaceRobotPose);
        LimelightPose.struct.pack(bb, value.BlueAllianceOriginFieldSpaceRobotPose);
        LimelightPose.struct.pack(bb, value.TargetSpaceRobotPose);
        LimelightPose.struct.pack(bb, value.TargetSpaceCameraPose);
        LimelightPose.struct.pack(bb, value.RobotSpaceCameraPose);
        LimelightPose.struct.pack(bb, value.CameraSpaceTargetPose);
        LimelightPose.struct.pack(bb, value.RobotSpaceTargetPose);
    }
}
