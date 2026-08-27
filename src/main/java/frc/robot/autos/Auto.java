package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import org.ironmaple.utils.FieldMirroringUtils;
import org.json.simple.parser.ParseException;

public interface Auto {
  /** Tunable delay to avoid collisions with partners at the start of Auto. */
  Command getAutoCommand(RobotContainer robot, double startDelay)
      throws IOException, ParseException;

  static Auto none() {
    return new Auto() {
      @Override
      public Command getAutoCommand(RobotContainer robot, double startDelay) {
        return Commands.none();
      }
    };
  }

  default Command setAutoStartPose(String pathName, Boolean mirrored, Drive drive) {
    final PathPlannerPath finalPath;
    try {
      PathPlannerPath loadedPath = PathPlannerPath.fromPathFile(pathName);
      if (mirrored) {
        loadedPath = loadedPath.mirrorPath();
      }
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        loadedPath = loadedPath.flipPath();
      }
      finalPath = loadedPath;
    } catch (Exception e) {
      DriverStation.reportError("Error: failed to load path: " + pathName, e.getStackTrace());
      return Commands.none();
    }
    return Commands.runOnce(() -> drive.setPose(finalPath.getStartingHolonomicPose().get()));
  }

  static PathPlannerPath getPath(String name, boolean mirror) throws IOException, ParseException {
    PathPlannerPath path = PathPlannerPath.fromPathFile(name);
    return mirror ? path.mirrorPath() : path;
  }

  static Pose2d flipLeftRight(Pose2d pose) {
    return new Pose2d(
        pose.getX(),
        FieldMirroringUtils.FIELD_HEIGHT - pose.getY(),
        pose.getRotation().unaryMinus());
  }

  default Command followPath(String pathName, boolean mirrored) {
    PathPlannerPath path;
    try {
      path = getPath(pathName, mirrored);
    } catch (Exception e) {
      DriverStation.reportError("Error: failed to load path: " + pathName, e.getStackTrace());
      return Commands.none();
    }
    return AutoBuilder.followPath(path);
  }
}
