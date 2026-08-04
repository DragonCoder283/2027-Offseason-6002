package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import java.io.IOException; // Added this import

public class AUTO_ReefscapeCoral extends SequentialCommandGroup {
  public AUTO_ReefscapeCoral(RobotContainer robot) {
    try {
      // This line throws the IOException that the compiler is complaining about
      // In your auto setup
      PathPlannerPath path = PathPlannerPath.fromPathFile("scorePreload");

      addCommands(
          new InstantCommand(() -> robot.drive.setPose(
              path.getStartingHolonomicPose().orElseThrow())),  // or handle Optional appropriately
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("scorePreload")),
            new WaitCommand(0.5),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("goToHp1")),
            new WaitCommand(0.5),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("scoreCoral1")),
            new WaitCommand(0.5),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("goToHp2")),
            new WaitCommand(0.5),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("scoreCoral2")),
            new WaitCommand(0.5),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("goToHp3")),
            new WaitCommand(0.5),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("scoreCoral3"))
      );
    } catch (IOException e) {
      // This catches the error gracefully so your code compiles and runs
      DriverStation.reportError(
          "PathPlanner file 'testPath' not found: " + e.getMessage(), e.getStackTrace());
    } catch (Exception e) {
      DriverStation.reportError("Generic error loading path: " + e.getMessage(), e.getStackTrace());
    }
  }
}
