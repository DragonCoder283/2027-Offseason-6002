package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AUTO_ChoreoTest extends SequentialCommandGroup {
    public AUTO_ChoreoTest(AutoFactory autoFactory, String path) {
        addCommands(
            autoFactory.resetOdometry(path),
            autoFactory.trajectoryCmd(path)
        );
    }
}
