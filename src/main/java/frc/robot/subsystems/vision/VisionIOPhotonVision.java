package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.Vision_Constants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();
        List<PhotonTrackedTarget> validTargets = new ArrayList<>();
        double totalDistance = 0.0;

        for (var target : result.targets) {
          double dist = target.bestCameraToTarget.getTranslation().getNorm();
          if (dist <= maxDistanceMeters && APRIL_TAG_MAP.containsKey(target.fiducialId)) {
            validTargets.add(target);
            totalDistance += dist;
          }
        }

        if (!validTargets.isEmpty()) {
          Transform3d fieldToCamera = multitagResult.estimatedPose.best;
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          for (var t : validTargets) tagIds.add((short) t.fiducialId);

          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  robotPose,
                  multitagResult.estimatedPose.ambiguity,
                  validTargets.size(),
                  totalDistance / validTargets.size(),
                  PoseObservationType.PHOTONVISION));
        }
      } else if (!result.targets.isEmpty()) {
        var target = result.targets.get(0);
        double distance = target.bestCameraToTarget.getTranslation().getNorm();

        if (distance <= maxDistanceMeters) {
          var tagPose = APRIL_TAG_MAP.get(target.fiducialId);

          if (tagPose != null) {
            Transform3d fieldToTarget =
                new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
            Transform3d cameraToTarget = target.bestCameraToTarget;

            Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            Pose3d robotPose =
                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            tagIds.add((short) target.fiducialId);

            poseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(),
                    robotPose,
                    target.poseAmbiguity,
                    1,
                    distance,
                    PoseObservationType.PHOTONVISION));
          }
        }
      }
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Short::intValue).toArray();
  }
}
