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

package frc.robot.subsystems.vision;

import static java.util.Map.entry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public class Vision_Constants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "LeftCam";
  public static String camera1Name = "RightCam";
  public static String camera2Name = "LeftSideCam";
  public static String camera3Name = "RightSideCam";

  public static final double stdDevFactor = 1;

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d robotToCamera0;
  public static final Transform3d robotToCamera1;
  public static final Transform3d robotToCamera2;
  public static final Transform3d robotToCamera3;

  static {
      robotToCamera0 =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(13.25),
                  Units.inchesToMeters(7.75),
                  Units.inchesToMeters(12)),
              new Rotation3d(0, Math.toRadians(-17), 0));

      robotToCamera1 =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(13.25),
                  Units.inchesToMeters(-7.75),
                  Units.inchesToMeters(12)),
              new Rotation3d(0, Math.toRadians(-17.75), 0));

        robotToCamera2 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(2.5),
                Units.inchesToMeters(-13.5),
                Units.inchesToMeters(6)),
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(90)));

    robotToCamera3 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(2.5),
                Units.inchesToMeters(13.5),
                Units.inchesToMeters(6)),
                new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-90)));      
    }

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static double maxZError = 0.125;
  public static double maxDistanceMeters = 5.15;

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

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // filter out all tags except the tags front, left, and right of both hubs
  public static final Map<Integer, Pose3d> APRIL_TAG_MAP =
      Map.ofEntries(
          entry(
              1,
              new Pose3d(
                  new Translation3d(11.8779798, 7.4247756, 0.889),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          entry(
              2,
              new Pose3d(
                  new Translation3d(11.9154194, 4.63804, 1.12395),
                  new Rotation3d(
                      new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
          entry(
              3,
              new Pose3d(
                  new Translation3d(11.3118646, 4.3902376, 1.12395),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          entry(
              4,
              new Pose3d(
                  new Translation3d(11.3118646, 4.0346376, 1.12395),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          entry(
              5,
              new Pose3d(
                  new Translation3d(11.9154194, 3.4312352, 1.12395),
                  new Rotation3d(
                      new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
          entry(
              6,
              new Pose3d(
                  new Translation3d(11.8779798, 0.6444996, 0.889),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          entry(
              7,
              new Pose3d(
                  new Translation3d(11.9528844, 0.6444996, 0.889),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          entry(
              8,
              new Pose3d(
                  new Translation3d(12.2710194, 3.4312352, 1.12395),
                  new Rotation3d(
                      new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
          entry(
              9,
              new Pose3d(
                  new Translation3d(12.5191774, 3.6790376, 1.12395),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          entry(
              10,
              new Pose3d(
                  new Translation3d(12.5191774, 4.0346376, 1.12395),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          entry(
              11,
              new Pose3d(
                  new Translation3d(12.2710194, 4.63804, 1.12395),
                  new Rotation3d(
                      new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
          entry(
              12,
              new Pose3d(
                  new Translation3d(11.9528844, 7.4247756, 0.889),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          //   entry(
          //       13,
          //       new Pose3d(
          //           new Translation3d(16.5333172, 7.4033126, 0.55245),
          //           new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          //   entry(
          //       14,
          //       new Pose3d(
          //           new Translation3d(16.5333172, 6.9715126, 0.55245),
          //           new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          //   entry(
          //       15,
          //       new Pose3d(
          //           new Translation3d(16.5329616, 4.3235626, 0.55245),
          //           new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          //   entry(
          //       16,
          //       new Pose3d(
          //           new Translation3d(16.5329616, 3.8917626, 0.55245),
          //           new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          entry(
              17,
              new Pose3d(
                  new Translation3d(4.6630844, 0.6444996, 0.889),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          entry(
              18,
              new Pose3d(
                  new Translation3d(4.6256194, 3.4312352, 1.12395),
                  new Rotation3d(
                      new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
          entry(
              19,
              new Pose3d(
                  new Translation3d(5.2291742, 3.6790376, 1.12395),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          entry(
              20,
              new Pose3d(
                  new Translation3d(5.2291742, 4.0346376, 1.12395),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          entry(
              21,
              new Pose3d(
                  new Translation3d(4.6256194, 4.63804, 1.12395),
                  new Rotation3d(
                      new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
          entry(
              22,
              new Pose3d(
                  new Translation3d(4.6630844, 7.4247756, 0.889),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          entry(
              23,
              new Pose3d(
                  new Translation3d(4.5881798, 7.4247756, 0.889),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          entry(
              24,
              new Pose3d(
                  new Translation3d(4.2700194, 4.63804, 1.12395),
                  new Rotation3d(
                      new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
          entry(
              25,
              new Pose3d(
                  new Translation3d(4.0218614, 4.3902376, 1.12395),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          entry(
              26,
              new Pose3d(
                  new Translation3d(4.0218614, 4.0346376, 1.12395),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          entry(
              27,
              new Pose3d(
                  new Translation3d(4.2700194, 3.4312352, 1.12395),
                  new Rotation3d(
                      new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
          entry(
              28,
              new Pose3d(
                  new Translation3d(4.5881798, 0.6444996, 0.889),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))))
          //   entry(
          //       29,
          //       new Pose3d(
          //           new Translation3d(0.007747, 0.6659626, 0.55245),
          //           new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          //   entry(
          //       30,
          //       new Pose3d(
          //           new Translation3d(0.007747, 1.0977626, 0.55245),
          //           new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          //   entry(
          //       31,
          //       new Pose3d(
          //           new Translation3d(0.0080772, 3.7457126, 0.55245),
          //           new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          //   entry(
          //       32,
          //       new Pose3d(
          //           new Translation3d(0.0080772, 4.1775126, 0.55245),
          //           new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))))
          );
}
