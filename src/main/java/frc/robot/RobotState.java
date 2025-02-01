// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/** Add your docs here. */
public class RobotState {
    private static RobotState instance;

    @Getter
    @Setter
    private Pose2d robotPose = new Pose2d();

    @Getter
    @Setter
    private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

    @RequiredArgsConstructor
    @Getter
    public enum TARGET {
        LEFT_CORAL_STATION(FieldConstants.CoralStation.leftCenterFace.getTranslation(), FieldConstants.CoralStation.leftCenterFace.getRotation()),
        RIGHT_CORAL_STATION(FieldConstants.CoralStation.rightCenterFace.getTranslation(), FieldConstants.CoralStation.rightCenterFace.getRotation()),
        REEF(FieldConstants.Reef.center, null),
        REEF_AB(FieldConstants.Reef.centerFaces[0].getTranslation(), FieldConstants.Reef.centerFaces[0].getRotation()),
        REEF_CD(FieldConstants.Reef.centerFaces[5].getTranslation(), FieldConstants.Reef.centerFaces[5].getRotation()),
        REEF_EF(FieldConstants.Reef.centerFaces[4].getTranslation(), FieldConstants.Reef.centerFaces[4].getRotation()),
        REEF_GH(FieldConstants.Reef.centerFaces[3].getTranslation(), FieldConstants.Reef.centerFaces[3].getRotation()),
        REEF_IJ(FieldConstants.Reef.centerFaces[2].getTranslation(), FieldConstants.Reef.centerFaces[2].getRotation()),
        REEF_KL(FieldConstants.Reef.centerFaces[1].getTranslation(), FieldConstants.Reef.centerFaces[1].getRotation()),    
        PROCESSOR(FieldConstants.Processor.centerFace.getTranslation(), FieldConstants.Processor.centerFace.getRotation()),
        BARGE(FieldConstants.Barge.middleCage, Rotation2d.fromDegrees(180));;

        private final Translation2d targetTranslation;
        private final Rotation2d targetRotation;
    }

    @Getter
    @Setter
    private TARGET target = TARGET.LEFT_CORAL_STATION;

    private double deltaT = .15;

    public static RobotState getInstance()
    {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    public Rotation2d getAngleToTarget(Translation2d currentTranslation)
    {
        return target.targetTranslation.minus(currentTranslation).getAngle();
    }

    // For Cardinal targets, such as the Processor, Net, Barge, and Coral Substations
    public Rotation2d getAngleOfTarget() {
        if (target.targetRotation == null) {
            return null;
        }
        // Return the angle to align to target
        return target.targetRotation;
    }

    public TARGET chooseReefTarget() {

        // Get the angle of the vector from the robot to the reef
        double angle = getAngleToTarget(TARGET.REEF.getTargetTranslation()).getDegrees();
    
        // Now get the angle of the nearest target
        if (((angle >= -30) && (angle < 30))) {
            return (TARGET.REEF_GH);
        } else if ((angle >= 30) && (angle < 90)) {
            return (TARGET.REEF_IJ);
        } else if ((angle >= 90) && (angle < 150)) {
            return (TARGET.REEF_KL);
        } else if (((angle >= 150) && (angle <= 180)) || ((angle > -180) && (angle < -150))) {
            return (TARGET.REEF_AB);
        } else if ((angle >= -150) && (angle < -90)) {
            return (TARGET.REEF_CD);
        } else if ((angle >= -90) && (angle < -30)) {
            return (TARGET.REEF_EF);
        } else {
          return (TARGET.REEF); // Catch all
        }
    }

    public double getDistanceToTarget(Pose2d currentPose) {
        return currentPose
            .getTranslation()
            .getDistance(target.targetTranslation);
    }

    private Translation2d getFuturePose()
    {
        // If magnitude of velocity is low enough, use current pose
        if (Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond) < .25) {
            return robotPose.getTranslation();
        } else {
            // Add translation based on current speed and time in the future deltaT
            return robotPose
                .getTranslation()
                .plus(
                    new Translation2d(
                        deltaT * robotSpeeds.vxMetersPerSecond,
                        deltaT * robotSpeeds.vyMetersPerSecond));
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
    
    public Command setTargetCommand(TARGET target) {
        return Commands.startEnd(() -> setTarget(target), () -> setTarget(TARGET.REEF))
            .withName("Set Robot Target: " + target.toString());
      }
}
