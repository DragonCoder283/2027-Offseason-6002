package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import java.io.IOException; // Added this import

public class AUTO_Test extends SequentialCommandGroup {
  public AUTO_Test(RobotContainer robot) {
    try {
      // This line throws the IOException that the compiler is complaining about
      // In your auto setup
      PathPlannerPath path = PathPlannerPath.fromPathFile("testPath");

      addCommands(
          AutoBuilder.resetOdom(
              path.getStartingDifferentialPose()), // or getStartingHolonomicPose()
              AutoBuilder.followPath(path));
    } catch (IOException e) {
      // This catches the error gracefully so your code compiles and runs
      DriverStation.reportError(
          "PathPlanner file 'testPath' not found: " + e.getMessage(), e.getStackTrace());
    } catch (Exception e) {
      DriverStation.reportError("Generic error loading path: " + e.getMessage(), e.getStackTrace());
    }
  }
}
