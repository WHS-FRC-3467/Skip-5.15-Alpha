// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Vision;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout;

    static {
        try {
            aprilTagLayout = new AprilTagFieldLayout(
                new File(Filesystem.getDeployDirectory(), "vision/andymark.json").toPath());
        } catch (IOException exception) {
            aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        }
    }

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "front_left";
    public static String camera1Name = "front_right";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.293688, 0.346515, 0.219053,
            new Rotation3d(0.0, Units.degreesToRadians(-15), 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(0.293688, -0.346515, 0.219053,
            new Rotation3d(0.0, Units.degreesToRadians(-15), 0.0));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
                1.0, // Camera 0
                1.0 // Camera 1
        };
}
