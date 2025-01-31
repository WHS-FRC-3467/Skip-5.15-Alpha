// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class RobotState {
  private static RobotState instance;

  @Getter @Setter private Pose2d robotPose = new Pose2d();

  @Getter @Setter private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

  @RequiredArgsConstructor
  @Getter
  public enum TARGET {
    // NONE(null, null),
    REEF_AB(Constants.FieldConstants.BLUE_REEF_AB, Constants.FieldConstants.RED_REEF_AB),
    REEF_CD(Constants.FieldConstants.BLUE_REEF_CD, Constants.FieldConstants.RED_REEF_CD),
    REEF_EF(Constants.FieldConstants.BLUE_REEF_EF, Constants.FieldConstants.RED_REEF_EF),
    REEF_GH(Constants.FieldConstants.BLUE_REEF_GH, Constants.FieldConstants.RED_REEF_GH),
    REEF_IJ(Constants.FieldConstants.BLUE_REEF_IJ, Constants.FieldConstants.RED_REEF_IJ),
    REEF_KL(Constants.FieldConstants.BLUE_REEF_KL, Constants.FieldConstants.RED_REEF_KL),
    REEF_CENTER(
        Constants.FieldConstants.BLUE_REEF_CENTER, Constants.FieldConstants.RED_REEF_CENTER),
    SUBSTATION_LEFT(
        Constants.FieldConstants.BLUE_SUBSTATION_LEFT,
        Constants.FieldConstants.RED_SUBSTATION_LEFT),
    SUBSTATION_RIGHT(
        Constants.FieldConstants.BLUE_SUBSTATION_RIGHT,
        Constants.FieldConstants.RED_SUBSTATION_RIGHT),
    PROCESSOR(Constants.FieldConstants.BLUE_PROCESSOR, Constants.FieldConstants.RED_PROCESSOR),
    NET(Constants.FieldConstants.BLUE_NET, Constants.FieldConstants.RED_NET),
    BARGE(Constants.FieldConstants.BLUE_BARGE, Constants.FieldConstants.RED_BARGE);

    // SPEAKER(Constants.FieldConstants.BLUE_SPEAKER,
    // Constants.FieldConstants.RED_SPEAKER),
    // AMP(Constants.FieldConstants.BLUE_AMP, Constants.FieldConstants.RED_AMP),
    // FEED(Constants.FieldConstants.BLUE_FEED, Constants.FieldConstants.RED_FEED);

    private final Pose2d blueTargetPose;
    private final Pose2d redTargetPose;
  }

  @Getter @Setter public TARGET target = TARGET.REEF_AB;

  // private double deltaT = .15;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/OdometryPose")
  private Pose2d odometryPose = new Pose2d();

  @Getter
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  private Pose2d estimatedPose = new Pose2d();

  // private Translation2d getFuturePose() {
  // // If magnitude of velocity is low enough, use current pose
  // if (Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
  // < .25) {
  // return robotPose.getTranslation();
  // } else {
  // // Add translation based on current speed and time in the future deltaT
  // return robotPose
  // .getTranslation()
  // .plus(
  // new Translation2d(
  // deltaT * robotSpeeds.vxMetersPerSecond, deltaT *
  // robotSpeeds.vyMetersPerSecond));
  // }
  // }

  public Alliance getAlliance() {
    try {
      return DriverStation.getAlliance().get();
    } catch (Exception e) {
      // TODO: Create a command setting alliance
      return Alliance.Red;
    }
  }

  // For Cardinal targets, such as the Processor, Net, Barge, and Coral
  // Substations
  public Rotation2d getAngleOfTarget() {
    // Return the angle to allign to target
    return (getAlliance() == Alliance.Blue)
        ? target.blueTargetPose.getRotation()
        : target.redTargetPose.getRotation();
  }

  // TODO: need to test or code review - for HEADING targets
  public Rotation2d getAngleToTarget() {
    // Get the angle of the vector from the robot to the reef
    return robotPose
        .getTranslation()
        .minus(
            (getAlliance() == Alliance.Blue)
                ? target.blueTargetPose.getTranslation()
                : target.redTargetPose.getTranslation()) // Get the vector from the robot to target
        // Robotpose - target
        .getAngle();
  }

  public double getAngleToTargetDegrees() {

    Transform2d m_transform =
        new Transform2d(
            robotPose,
            (getAlliance() == Alliance.Blue) ? target.blueTargetPose : target.redTargetPose);
    double angle = Math.toDegrees(Math.atan(m_transform.getY() / m_transform.getX()));
    return angle;
  }

  public TARGET chooseReefTarget() {

    // Get the angle of the vector from the robot to the reef
    double angle =
        robotPose
            .getTranslation()
            .minus(
                (getAlliance() == Alliance.Blue)
                    ? Constants.FieldConstants.BLUE_REEF_CENTER.getTranslation()
                    : Constants.FieldConstants.RED_REEF_CENTER
                        .getTranslation()) // Get the vector from the robot to the reef.
            // Robotpose - reef center
            .getAngle()
            .getDegrees();

    // Now get the angle of the nearest target
    if (((angle >= -30) && (angle < 30))) {
      if (getAlliance() == Alliance.Blue) {
        return (TARGET.REEF_GH);
      } else {
        return (TARGET.REEF_AB);
      }
    } else if ((angle >= 30) && (angle < 90)) {
      if (getAlliance() == Alliance.Blue) {
        return (TARGET.REEF_IJ);
      } else {
        return (TARGET.REEF_CD);
      }
    } else if ((angle >= 90) && (angle < 150)) {
      if (getAlliance() == Alliance.Blue) {
        return (TARGET.REEF_KL);
      } else {
        return (TARGET.REEF_EF);
      }
    } else if (((angle >= 150) && (angle <= 180)) || ((angle > -180) && (angle < -150))) {
      if (getAlliance() == Alliance.Blue) {
        return (TARGET.REEF_AB);
      } else {
        return (TARGET.REEF_GH);
      }
    } else if ((angle >= -150) && (angle < -90)) {
      if (getAlliance() == Alliance.Blue) {
        return (TARGET.REEF_CD);
      } else {
        return (TARGET.REEF_IJ);
      }
    } else if ((angle >= -90) && (angle < -30)) {
      if (getAlliance() == Alliance.Blue) {
        return (TARGET.REEF_EF);
      } else {
        return (TARGET.REEF_KL);
      }
    } else {
      return (TARGET.REEF_CENTER); // Catch all
    }
  }

  // todo: need to invert
  // public Rotation2d getAngleToTarget() {
  // return getFuturePose()
  // .minus(
  // (getAlliance() == Alliance.Blue)
  // ? target.blueTargetPose.getTranslation()
  // : target.redTargetPose.getTranslation())
  // .getAngle()
  // .unaryMinus(); // todo: Test if unaryMinus fixed it
  // }

  // private double getDistanceToTarget() {
  // return getFuturePose()
  // .getDistance(
  // (getAlliance() == Alliance.Blue)
  // ? target.blueTargetPose.getTranslation()
  // : target.redTargetPose.getTranslation());
  // }

  private double getDistanceToTarget() {
    return robotPose
        .getTranslation()
        .getDistance(
            (getAlliance() == Alliance.Blue)
                ? target.blueTargetPose.getTranslation()
                : target.redTargetPose.getTranslation());
  }

  // private static final InterpolatingDoubleTreeMap speakerArmAngleMap = new
  // InterpolatingDoubleTreeMap();

  // static {
  // speakerArmAngleMap.put(1.5, 12.71);
  // speakerArmAngleMap.put(2.0, 21.00);
  // speakerArmAngleMap.put(2.5, 24.89);
  // speakerArmAngleMap.put(3.0, 29.00);
  // speakerArmAngleMap.put(3.5, 31.20);
  // speakerArmAngleMap.put(4.0, 32.50);
  // speakerArmAngleMap.put(4.5, 34.00);
  // speakerArmAngleMap.put(5.0, 35.00);
  // }

  // private static final InterpolatingDoubleTreeMap feedArmAngleMap = new
  // InterpolatingDoubleTreeMap();

  // static {
  // feedArmAngleMap.put(5.0, 0.0);
  // feedArmAngleMap.put(6.0, -10.0);
  // feedArmAngleMap.put(7.0, -19.0);
  // }

  // public double getShotAngle() {
  // switch (target) {
  // case SPEAKER:
  // return speakerArmAngleMap.get(getDistanceToTarget());
  // case FEED:
  // return feedArmAngleMap.get(getDistanceToTarget());
  // default:
  // return 0.0;
  // }
  // }

  public Command setTargetCommand(TARGET target) {
    return Commands.startEnd(() -> setTarget(target), () -> setTarget(TARGET.REEF_EF))
        .withName("Set Robot Target: " + target.toString());
  }
}
