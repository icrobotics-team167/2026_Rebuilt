// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.List;
import org.photonvision.targeting.*;

public class PhotonStructs {
  public static final Struct<PhotonPipelineMetadata> metadataStruct =
      new Struct<>() {
        @Override
        public Class<PhotonPipelineMetadata> getTypeClass() {
          return PhotonPipelineMetadata.class;
        }

        @Override
        public String getTypeName() {
          return "PhotonPipelineMetadata";
        }

        @Override
        public int getSize() {
          return kSizeInt64 * 4;
        }

        @Override
        public String getSchema() {
          return "int64 captureTimestampMicros;"
              + "int64 publishTimestampMicros;"
              + "int64 sequenceID;"
              + "int64 timeSinceLastPong";
        }

        @Override
        public PhotonPipelineMetadata unpack(ByteBuffer bb) {
          return new PhotonPipelineMetadata(bb.getLong(), bb.getLong(), bb.getLong(), bb.getLong());
        }

        @Override
        public void pack(ByteBuffer bb, PhotonPipelineMetadata value) {
          bb.putLong(value.captureTimestampMicros);
          bb.putLong(value.publishTimestampMicros);
          bb.putLong(value.sequenceID);
          bb.putLong(value.timeSinceLastPong);
        }
      };

  public static final Struct<PhotonTrackedTarget> trackedTargetStruct =
      new Struct<>() {
        @Override
        public Class<PhotonTrackedTarget> getTypeClass() {
          return PhotonTrackedTarget.class;
        }

        @Override
        public String getTypeName() {
          return "PhotonTrackedTarget";
        }

        @Override
        public int getSize() {
          return kSizeDouble * 4
              + kSizeInt32 * 2
              + kSizeFloat
              + Transform3d.struct.getSize() * 2
              + kSizeDouble
              + targetCornerStruct.getSize() * 4
              // Technically this should be 8 since detectedCorners can have up to 8 corners,
              // But if an AprilTag doesn't have 4 corners, something has gone very wrong.
              + targetCornerStruct.getSize() * 4;
        }

        @Override
        public String getSchema() {
          return "double yaw;double pitch;double area;double skew;"
              + "int32 fiducialId;int32 objDetectId;float objDetectConf;"
              + "Transform3d bestCameraToTarget;Transform3d altCameraToTarget;double poseAmbiguity;"
              + "TargetCorner minAreaRectCorners[4];TargetCorner detectedCorners[4]";
        }

        @Override
        public PhotonTrackedTarget unpack(ByteBuffer bb) {
          return new PhotonTrackedTarget(
              bb.getDouble(), // yaw
              bb.getDouble(), // pitch
              bb.getDouble(), // area
              bb.getDouble(), // skew
              bb.getInt(), // fiducialId
              bb.getInt(), // objDetectId
              bb.getFloat(), // objDetectConf
              Transform3d.struct.unpack(bb), // bestCameraToTarget
              Transform3d.struct.unpack(bb), // altCameraToTarget
              bb.getDouble(), // poseAmbiguity
              List.of(Struct.unpackArray(bb, 4, targetCornerStruct)), // minAreaRectCorners
              List.of(Struct.unpackArray(bb, 4, targetCornerStruct))); // detectedCorners
        }

        @Override
        public void pack(ByteBuffer bb, PhotonTrackedTarget value) {
          bb.putDouble(value.yaw);
          bb.putDouble(value.pitch);
          bb.putDouble(value.area);
          bb.putDouble(value.skew);
          bb.putInt(value.fiducialId);
          bb.putInt(value.objDetectId);
          bb.putFloat(value.objDetectConf);
          Transform3d.struct.pack(bb, value.bestCameraToTarget);
          Transform3d.struct.pack(bb, value.altCameraToTarget);
          bb.putDouble(value.poseAmbiguity);
          Struct.packArray(
              bb, value.getMinAreaRectCorners().toArray(new TargetCorner[4]), targetCornerStruct);
          Struct.packArray(
              bb, value.getDetectedCorners().toArray(new TargetCorner[4]), targetCornerStruct);
        }

        @Override
        public Struct<?>[] getNested() {
          return new Struct<?>[] {Transform3d.struct, targetCornerStruct};
        }
      };

  public static final Struct<TargetCorner> targetCornerStruct =
      new Struct<>() {
        @Override
        public Class<TargetCorner> getTypeClass() {
          return TargetCorner.class;
        }

        @Override
        public String getTypeName() {
          return "TargetCorner";
        }

        @Override
        public int getSize() {
          return kSizeDouble * 2;
        }

        @Override
        public String getSchema() {
          return "double x;double y";
        }

        @Override
        public TargetCorner unpack(ByteBuffer bb) {
          return new TargetCorner(bb.getDouble(), bb.getDouble());
        }

        @Override
        public void pack(ByteBuffer bb, TargetCorner value) {
          bb.putDouble(value.x);
          bb.putDouble(value.y);
        }
      };

  public static Struct<PnpResult> pnpResultStruct =
      new Struct<>() {
        @Override
        public Class<PnpResult> getTypeClass() {
          return PnpResult.class;
        }

        @Override
        public String getTypeName() {
          return "PnpResult";
        }

        @Override
        public int getSize() {
          return Transform3d.struct.getSize() * 2 + kSizeDouble * 3;
        }

        @Override
        public String getSchema() {
          return "Transform3d best;Transform3d alt;double ambiguity;double bestReprojErr;"
              + "double altReprojErr";
        }

        @Override
        public PnpResult unpack(ByteBuffer bb) {
          return new PnpResult(
              Transform3d.struct.unpack(bb), // best
              Transform3d.struct.unpack(bb), // alt
              bb.getDouble(), // ambiguity
              bb.getDouble(), // bestReprojErr
              bb.getDouble()); // altReprojErr
        }

        @Override
        public void pack(ByteBuffer bb, PnpResult value) {
          Transform3d.struct.pack(bb, value.best);
          Transform3d.struct.pack(bb, value.alt);
          bb.putDouble(value.ambiguity);
          bb.putDouble(value.bestReprojErr);
          bb.putDouble(value.altReprojErr);
        }

        @Override
        public Struct<?>[] getNested() {
          return new Struct<?>[] {Transform3d.struct};
        }
      };
}
