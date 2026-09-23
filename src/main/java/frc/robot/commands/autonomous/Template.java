package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;
import org.wpilib.command2.SequentialCommandGroup;

public class Template extends SequentialCommandGroup {
  public Template() {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath startingPath = PathPlannerPath.fromChoreoTrajectory("NewPath");
      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      Pose2d startingPose =
          new Pose2d(
              startingPath.getPoint(0).position, startingPath.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(startingPath)
          /*
           * TODO: The rest of the autonomous routine command
           */
          );

    } catch (Exception e) {
      DriverStationErrors.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
