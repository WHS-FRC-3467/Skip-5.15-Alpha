/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.PhotonVision.PhotonVisionConstants.*;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

// Taken from
// https://github.com/gladiatorsprogramming1591/Crescendo2024/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java#L794

public class PhotonVision extends SubsystemBase {
  private Drive drivetrain;
  private PhotonPoseEstimator[] m_photonPoseEstimators;
  private PhotonCamera m_frontLeftCamera;
  // private PhotonCamera m_frontRightCamera;
  AprilTagFieldLayout fieldLayout;

  private Field2d visionPose = new Field2d();

  public PhotonVision(Drive drivetrain) {
    this.drivetrain = drivetrain;

    fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    m_frontLeftCamera = new PhotonCamera(front_left_cam.kCameraName);
    // m_frontRightCamera = new PhotonCamera(front_right_cam.kCameraName);
    m_photonPoseEstimators =
        new PhotonPoseEstimator[] {
          new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              front_left_cam.kRobotToCam),
          /*                    new PhotonPoseEstimator(
          AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          m_frontRightCamera,
          front_right_cam.kRobotToCam) */
        };

    SmartDashboard.putData("PhotonVisoin Pose", visionPose);
  }

  @Override
  public void periodic() {
    Pose2d currentPose = RobotState.getInstance().getRobotPose();
    for (PhotonPoseEstimator poseEstimator : m_photonPoseEstimators) {
      // print out the time for this line to run
      Optional<EstimatedRobotPose> pose = poseEstimator.update(
        m_frontLeftCamera.getAllUnreadResults().get(m_frontLeftCamera.getAllUnreadResults().size() - 1));
      if (pose.isPresent()) {
        Pose3d pose3d = pose.get().estimatedPose;
        Pose2d pose2d = pose3d.toPose2d();
        if (pose3d.getX() >= -PhotonVisionConstants.VISION_XY_MARGIN
            && pose3d.getX()
                <= FieldConstants.FIELD_X_LENGTH + PhotonVisionConstants.VISION_XY_MARGIN
            && pose3d.getY() >= -PhotonVisionConstants.VISION_XY_MARGIN
            && pose3d.getY()
                <= FieldConstants.FIELD_Y_LENGTH + PhotonVisionConstants.VISION_XY_MARGIN
            && pose3d.getZ() >= -PhotonVisionConstants.VISION_Z_MARGIN
            && pose3d.getZ() <= PhotonVisionConstants.VISION_Z_MARGIN) {
          double sum = 0.0;
          for (PhotonTrackedTarget target : pose.get().targetsUsed) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            sum +=
                currentPose
                    .getTranslation()
                    .getDistance(tagPose.get().getTranslation().toTranslation2d());
          }

          int tagCount = pose.get().targetsUsed.size();
          double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
          double xyStd = PhotonVisionConstants.VISION_STD_XY_SCALE * stdScale;
          double rotStd = PhotonVisionConstants.VISION_STD_ROT_SCALE * stdScale;

          drivetrain.addVisionMeasurement(
              pose2d, pose.get().timestampSeconds, VecBuilder.fill(xyStd, xyStd, rotStd));
          visionPose.setRobotPose(pose2d);
          continue;
        }
      }
    }
  }
}
