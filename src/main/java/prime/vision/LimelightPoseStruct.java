package prime.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;

public class LimelightPoseStruct implements Struct<LimelightPose> {
    @Override
    public Class<LimelightPose> getTypeClass() {
        return LimelightPose.class;
    }

    @Override
    public String getTypeName() {
        return "LimelightPose";
    }

    @Override
    public int getSize() {
        var size = Pose3d.struct.getSize() + (kSizeDouble * 5) + (kSizeDouble * 3);

        return size;
    }

    @Override
    public String getSchema() {
        return "Pose3d pose;double timestamp;double tagCount;double tagSpan;double avgTagDistanceMeters;double avgTagArea;double stdDeviations[3]";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {Pose3d.struct};
    }

    @Override
    public LimelightPose unpack(ByteBuffer bb) {
        var pose = Pose3d.struct.unpack(bb);
        var tagData = Struct.unpackDoubleArray(bb, 5);
        var stdDeviations = Struct.unpackDoubleArray(bb, 3);

        return new LimelightPose(pose, tagData, stdDeviations);
    }

    @Override
    public void pack(ByteBuffer bb, LimelightPose value) {
        Pose3d.struct.pack(bb, value.Pose); 
        bb.putDouble(value.Timestamp);
        bb.putDouble(value.TagCount);
        bb.putDouble(value.TagSpan);
        bb.putDouble(value.AvgTagDistanceMeters);
        bb.putDouble(value.AvgTagArea);
        Struct.packArray(bb, value.StdDeviations);
    }
}
