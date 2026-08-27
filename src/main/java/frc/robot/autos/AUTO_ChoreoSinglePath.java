package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AUTO_ChoreoSinglePath extends SequentialCommandGroup {
    public AUTO_ChoreoSinglePath(AutoFactory autoFactory, String path) {
        addCommands(
            autoFactory.resetOdometry(path),
            autoFactory.trajectoryCmd(path)
        );
    }
}
