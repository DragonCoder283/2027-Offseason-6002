package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import java.io.IOException; // Added this import

public class AUTO_Reefscape extends SequentialCommandGroup {
  public AUTO_Reefscape(RobotContainer robot) {
    try {
      // This line throws the IOException that the compiler is complaining about
      // In your auto setup
      PathPlannerPath path = PathPlannerPath.fromPathFile("goToReef");

      addCommands(
          new InstantCommand(() -> robot.drive.setPose(
              path.getStartingHolonomicPose().orElseThrow())),  // or handle Optional appropriately
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("goToReef")),
            new WaitCommand(0.25),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("pickAlgae1")),
            new WaitCommand(0.25),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("scoreAlgae1")),
            new WaitCommand(0.25),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("pickAlgae2")),
            new WaitCommand(0.25),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("scoreAlgae2"))
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
