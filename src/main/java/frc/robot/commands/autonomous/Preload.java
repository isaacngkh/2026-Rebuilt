package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.framework.RobotBase;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.driverstation.DriverStationErrors;

import frc.robot.EagleUtil;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Preload extends SequentialCommandGroup {
  public Preload(SwerveSubsystem drivetrain, ShooterSubsystem shooter, IndexerSubsystem indexer) {

    try {

      Pose2d startingPose = new Pose2d(3.51, 4.03, new Rotation2d(0));

      addCommands(
          AutoBuilder.resetOdom(startingPose),
          Commands.waitSeconds(5)
              .deadlineFor(
                  indexer.index(),
                  shooter.cruiseControl(),
                  EagleUtil.shootInSim(drivetrain).onlyIf(() -> RobotBase.isSimulation())),
          Commands.parallel(shooter.runVoltage(0), indexer.runVoltage(0)));

    } catch (Exception e) {
      DriverStationErrors.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
