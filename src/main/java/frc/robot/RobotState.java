// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/** Add your docs here. */
public class RobotState {
  private static RobotState instance;

  @Getter @Setter private Pose2d robotPose = new Pose2d();

  @Getter @Setter private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

  @RequiredArgsConstructor
  @Getter
  public enum TARGET {
    LEFT_CORAL_STATION(FieldConstants.CoralStation.leftCenterFace.getTranslation()),
    RIGHT_CORAL_STATION(FieldConstants.CoralStation.rightCenterFace.getTranslation()),
    REEF(FieldConstants.Reef.center);

    private final Translation2d TargetPose;
  }

  @Getter @Setter private TARGET target = TARGET.LEFT_CORAL_STATION;

  private double deltaT = .15;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public TARGET getNextTarget() {
    TARGET[] values = TARGET.values();
    int index = target.ordinal();
    int nextIndex = (index + 1) % values.length;
    return values[nextIndex];
  }

  public Rotation2d getAngleToTarget(Pose2d currentPose) {
    return target.TargetPose.minus(currentPose.getTranslation()).getAngle();
  }

  private Translation2d getFuturePose() {
    // If magnitude of velocity is low enough, use current pose
    if (Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond) < .25) {
      return robotPose.getTranslation();
    } else {
      // Add translation based on current speed and time in the future deltaT
      return robotPose
          .getTranslation()
          .plus(
              new Translation2d(
                  deltaT * robotSpeeds.vxMetersPerSecond, deltaT * robotSpeeds.vyMetersPerSecond));
    }
  }

  private static final InterpolatingDoubleTreeMap speakerArmAngleMap =
      new InterpolatingDoubleTreeMap();

  static {
    speakerArmAngleMap.put(1.5, 12.71);
    speakerArmAngleMap.put(2.0, 21.00);
    speakerArmAngleMap.put(2.5, 24.89);
    speakerArmAngleMap.put(3.0, 29.00);
    speakerArmAngleMap.put(3.5, 31.20);
    speakerArmAngleMap.put(4.0, 32.50);
    speakerArmAngleMap.put(4.5, 34.00);
    speakerArmAngleMap.put(5.0, 35.00);
  }

  private static final InterpolatingDoubleTreeMap feedArmAngleMap =
      new InterpolatingDoubleTreeMap();

  static {
    feedArmAngleMap.put(5.0, 0.0);
    feedArmAngleMap.put(6.0, -10.0);
    feedArmAngleMap.put(7.0, -19.0);
  }
}
