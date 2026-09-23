package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.system.RobotController;
import org.wpilib.framework.TimedRobot;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private double prevTime = RobotController.getTime();
  private final GcStatsCollector gcStatsCollector = new GcStatsCollector();

  public Robot() {
    // Setup DogLog
    DogLog.setOptions(
        new DogLogOptions());
    DogLog.setPdh(new PowerDistribution(0));

    m_robotContainer = new RobotContainer(this::addPeriodic);

    DogLog.log("/Metadata/Branch", BuildConstants.GIT_BRANCH);
    DogLog.log("/Metadata/SHA", BuildConstants.GIT_SHA);
  }

  @Override
  public void robotPeriodic() {
    long startTime = RobotController.getTime();
    CommandScheduler.getInstance().run();
    DogLog.log("Loop Time/Command Scheduler", (RobotController.getTime() - startTime) / 1000);
    long endTime = RobotController.getTime();
    m_robotContainer.periodic();
    DogLog.log("Loop Time/Robot Container", (RobotController.getTime() - endTime) / 1000);

    gcStatsCollector.update();

    long currentTime = RobotController.getTime();
    DogLog.log("Loop Time/Total", (currentTime - prevTime) / 1000);
    prevTime = currentTime;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void simulationInit() {}

  private static final class GcStatsCollector {
    private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final long[] lastTimes = new long[gcBeans.size()];
    private final long[] lastCounts = new long[gcBeans.size()];

    public void update() {
      long accumTime = 0;
      long accumCounts = 0;
      for (int i = 0; i < gcBeans.size(); i++) {
        long gcTime = gcBeans.get(i).getCollectionTime();
        long gcCount = gcBeans.get(i).getCollectionCount();
        accumTime += gcTime - lastTimes[i];
        accumCounts += gcCount - lastCounts[i];

        lastTimes[i] = gcTime;
        lastCounts[i] = gcCount;
      }

      DogLog.log("GC/GCTimeMS", (double) accumTime);
      DogLog.log("GC/GCCounts", (double) accumCounts);
    }
  }

  @Override
  public void simulationPeriodic() {
    FuelSim.getInstance().updateSim();
    int bhs = FuelSim.Hub.BLUE_HUB.getScore(); // get number of fuel scored in blue hub
    int rhs = FuelSim.Hub.RED_HUB.getScore(); // get number of fuel scored in red hub
    DogLog.log("blue hub score", bhs);
    DogLog.log("red hub score", rhs);

    double fuelInHopper = FuelSim.getFuelInHopper();
    DogLog.log("number of fuels in hopper", fuelInHopper);
  }
}
