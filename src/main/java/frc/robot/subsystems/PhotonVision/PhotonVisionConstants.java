package frc.robot.subsystems.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class PhotonVisionConstants {
  public static class front_left_cam {
    public static final String kCameraName = "front_left";
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(-0.09, 0.170, 0.628),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-15)));
  }

  public static class front_right_cam {
    public static final String kCameraName = "front_right";
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(-0.09, -0.170, 0.628),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(15)));
  }

  public static class back_right_cam {
    public static final String kCameraName = "back_right";
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(-0.143, -0.321, 0.534),
            new Rotation3d(0, 0, Units.degreesToRadians(-155)));
  }

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static final double VISION_XY_MARGIN = 0.5;
  public static final double VISION_Z_MARGIN = 0.75;
  public static final double VISION_STD_XY_SCALE = 0.02 * 10;
  public static final double VISION_STD_ROT_SCALE = 0.035 * 10;
}
