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

import static frc.robot.subsystems.vision.Vision_Constants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  public Vision(
      VisionConsumer consumer, Supplier<ChassisSpeeds> chassisSpeedsSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Get current chassis speed for scaling trust
    double chassisSpeed =
        Math.abs(
            Math.hypot(
                chassisSpeedsSupplier.get().vxMetersPerSecond,
                chassisSpeedsSupplier.get().vyMetersPerSecond));

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate
                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // || chassisSpeed > 1
        // || chassisRotSpeed > Math.toRadians(15);

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0)
                * Vision_Constants.stdDevFactor
                / observation.tagCount();

        // Scale trust based on chassis velocity
        // Higher speeds increase the standard deviation (trusting vision less)
        double velocityMultiplier = 1.0 + (chassisSpeed / DriveConstants.maxSpeedMetersPerSec);
        stdDevFactor *= velocityMultiplier;

        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        Logger.recordOutput("Vision/LinearStdDev", linearStdDev);
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        Logger.recordOutput("Vision/AngStdDev", angularStdDev);

        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public Pose3d lastResult(SwerveDriveSimulation driveSimulation, int cameraIndex) {
    List<Pose3d> theTagPoses = new ArrayList<>();
    int tagIndexOfClosestTag = 0;
    double tagDistanceFromRobot = 1000;
    double closestTagDistanceFromRobot = tagDistanceFromRobot;
    Pose3d currentTag;
    Pose2d currentRobotPose = driveSimulation.getSimulatedDriveTrainPose();

    for (int i = 0; i < inputs[cameraIndex].tagIds.length; i++) {
      currentTag = aprilTagLayout.getTagPose(inputs[cameraIndex].tagIds[i]).get();
      theTagPoses.add(currentTag);
      tagDistanceFromRobot =
          currentRobotPose
              .getTranslation()
              .getDistance(currentTag.getTranslation().toTranslation2d());

      if (tagDistanceFromRobot < closestTagDistanceFromRobot) {
        closestTagDistanceFromRobot = tagDistanceFromRobot;
        tagIndexOfClosestTag = i;
      }
    }

    if (!theTagPoses.isEmpty()) {
      return theTagPoses.get(tagIndexOfClosestTag);
    } else {
      return null;
    }
  }

  public double lastResultDistance(Drive drive, int cameraIndex) {
    List<Pose3d> theTagPoses = new ArrayList<>();
    double tagDistanceFromRobot = 1000;
    double closestTagDistanceFromRobot = tagDistanceFromRobot;
    Pose3d currentTag;
    Transform3d robotToCamera = cameraIndex == 0 ? robotToCamera0 : robotToCamera1;
    Pose2d currentRobotPose =
        drive
            .getPose()
            .plus(
                new Transform2d(
                    robotToCamera.getTranslation().toTranslation2d(), new Rotation2d()));

    for (int i = 0; i < inputs[cameraIndex].tagIds.length; i++) {
      currentTag = aprilTagLayout.getTagPose(inputs[cameraIndex].tagIds[i]).get();
      theTagPoses.add(currentTag);
      tagDistanceFromRobot =
          currentRobotPose
              .getTranslation()
              .getDistance(currentTag.getTranslation().toTranslation2d());

      if (tagDistanceFromRobot < closestTagDistanceFromRobot) {
        closestTagDistanceFromRobot = tagDistanceFromRobot;
      }
    }

    return closestTagDistanceFromRobot;
  }

  public int lastResultId(SwerveDriveSimulation driveSimulation, int cameraIndex) {
    List<Pose3d> theTagPoses = new ArrayList<>();
    int tagIndexOfClosestTag = 0;
    double tagDistanceFromRobot = 1000;
    double closestTagDistanceFromRobot = tagDistanceFromRobot;
    Pose3d currentTag;
    Pose2d currentRobotPose = driveSimulation.getSimulatedDriveTrainPose();

    for (int i = 0; i < inputs[cameraIndex].tagIds.length; i++) {
      currentTag = aprilTagLayout.getTagPose(inputs[cameraIndex].tagIds[i]).get();
      theTagPoses.add(currentTag);
      tagDistanceFromRobot =
          currentRobotPose
              .getTranslation()
              .getDistance(currentTag.getTranslation().toTranslation2d());

      if (tagDistanceFromRobot < closestTagDistanceFromRobot) {
        closestTagDistanceFromRobot = tagDistanceFromRobot;
        tagIndexOfClosestTag = i;
      }
    }

    if (!theTagPoses.isEmpty()) {
      return tagIndexOfClosestTag;
    } else {
      return 0;
    }
  }

  /**
   * @param drive
   * @return Returns the last result of the vision in the real world.
   */
  public Pose2d lastResult(Drive drive, int cameraIndex) {
    List<Pose3d> theTagPoses = new ArrayList<>();
    int tagIndexOfClosestTag = 0;
    double tagDistanceFromRobot = 1000;
    double closestTagDistanceFromRobot = tagDistanceFromRobot;
    Pose3d currentTag;
    Pose2d currentRobotPose = drive.getPose();

    for (int i = 0; i < inputs[cameraIndex].tagIds.length; i++) {
      currentTag = aprilTagLayout.getTagPose(inputs[cameraIndex].tagIds[i]).get();
      theTagPoses.add(currentTag);
      tagDistanceFromRobot =
          currentRobotPose
              .getTranslation()
              .getDistance(currentTag.getTranslation().toTranslation2d());

      if (tagDistanceFromRobot < closestTagDistanceFromRobot) {
        closestTagDistanceFromRobot = tagDistanceFromRobot;
        tagIndexOfClosestTag = i;
      }
    }

    if (!theTagPoses.isEmpty()) {
      return theTagPoses.get(tagIndexOfClosestTag).toPose2d();
    } else {
      return null;
    }
  }

  public Pose3d getRobotPoseEstimator(SwerveDriveSimulation sim, int index) {
    Pose3d robotPose = new Pose3d();
    int targetId = lastResultId(sim, index);

    if (aprilTagLayout.getTagPose(targetId).isPresent()) {
      PhotonTrackedTarget target =
          new VisionIOPhotonVisionSim(
                  Vision_Constants.camera0Name,
                  Vision_Constants.robotToCamera0,
                  sim::getSimulatedDriveTrainPose)
              .camera
              .getLatestResult()
              .getBestTarget();
      robotPose =
          PhotonUtils.estimateFieldToRobotAprilTag(
              target.bestCameraToTarget,
              aprilTagLayout.getTagPose(targetId).get(),
              Vision_Constants.robotToCamera0);
    }

    return robotPose;
  }
}
