package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.pathplanner.lib.commands.FollowPathCommand;
import dev.doglog.DogLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alert;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.RobotController;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.DelayAuto;
import frc.robot.commands.autonomous.DepotPathAuto_1c;
import frc.robot.commands.autonomous.NeutralAutos;
import frc.robot.commands.autonomous.NeutralAutos.Routine;
import frc.robot.commands.autonomous.Preload;
import frc.robot.subsystems.aprilTagCam.AprilTagCam;
import frc.robot.subsystems.aprilTagCam.AprilTagCamConstants;
import frc.robot.subsystems.blocker.BlockerSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.objectDetection.GamePieceTracker;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;
import java.util.Optional;
import java.util.function.BiConsumer;

public class RobotContainer {
  public enum Robot {
    DEV,
    COMP,
    ANEMONE,
    KITBOT,
    SIM
  }

  @SuppressWarnings("resource")
  public static Robot getRobot() {
    final String serialNumber = RobotController.getSerialNumber();
    if (RobotBase.isSimulation()) {
      return Robot.SIM;
    } else if (serialNumber.equals("032414F0")) {
      return Robot.ANEMONE;
    } else if (serialNumber.equals("88888")) {
      return Robot.DEV;
    } else if (serialNumber.equals("03223849")) {
      return Robot.COMP;
    } else if (serialNumber.equals("0323CA18")) {
      return Robot.KITBOT;
    } else {
      new Alert(
              "roborio unrecognized. here is the serial number:" + serialNumber,
              Alert.Level.HIGH)
          .set(true);
      ;
      return Robot.COMP;
    }
  }

  // private ObjectDetectionCam objDecCam;

  private final BiConsumer<Runnable, Double> addPeriodic;

  private final CANBus canBus0 = CANBus.systemcore(0);
  private final CANBus canBus1 = CANBus.systemcore(1);
  private final CANBus canBus2 = CANBus.systemcore(2);
  private final CANBus canBus3 = CANBus.systemcore(3);
  private final CANBus canBus4 = CANBus.systemcore(4);
  private final CANBus canivoreCanbus = new CANBus("CAN_Network");

  private final CommandNiDsXboxController controller = new CommandNiDsXboxController(0);
  
  private final SwerveSubsystem drivetrain = new SwerveSubsystem(controller);
  private final ShooterSubsystem shooter = new ShooterSubsystem(canBus2, drivetrain.poseSupplier(), drivetrain::getVirtualTarget);
  private final GroundIntakeRollerSubsystem groundIntakeRoller = new GroundIntakeRollerSubsystem(canBus4);
  private final GroundIntakeLinearExtensionSubsystem groundIntakeExtension = new GroundIntakeLinearExtensionSubsystem(canBus3);
  private final ClimberSubsystem climber = new ClimberSubsystem(canBus0);
  private final IndexerSubsystem indexer = new IndexerSubsystem(canBus1);
  private final BlockerSubsystem blocker = new BlockerSubsystem(canBus0);

  private final DriveCommand defualtDriveCommand = new DriveCommand(drivetrain, controller);

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(groundIntakeExtension);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public final Trigger isHubActive = new Trigger(() -> HubTracker.isHubActiveCustom());

  private AprilTagCam[] cameras = {
              new AprilTagCam(
                AprilTagCamConstants.BACK_RIGHT_CAM,
                AprilTagCamConstants.BACK_RIGHT_CAM_LOCATION,
                drivetrain::addVisionMeasurement,
                () -> drivetrain.getCachedState().Pose,
                () -> drivetrain.getCachedState().Velocity),
              new AprilTagCam(
                AprilTagCamConstants.BACK_LEFT_CAM,
                AprilTagCamConstants.BACK_LEFT_CAM_LOCATION,
                drivetrain::addVisionMeasurement,
                () -> drivetrain.getCachedState().Pose,
                () -> drivetrain.getCachedState().Velocity),
              new AprilTagCam(
                AprilTagCamConstants.FRONT_LEFT_CAM,
                AprilTagCamConstants.FRONT_LEFT_CAM_LOCATION,
                drivetrain::addVisionMeasurement,
                () -> drivetrain.getCachedState().Pose,
                () -> drivetrain.getCachedState().Velocity),
              new AprilTagCam(
                AprilTagCamConstants.FRONT_RIGHT_CAM,
                AprilTagCamConstants.FRONT_RIGHT_CAM_LOCATION,
                drivetrain::addVisionMeasurement,
                () -> drivetrain.getCachedState().Pose,
                () -> drivetrain.getCachedState().Velocity)
              };

  public RobotContainer(BiConsumer<Runnable, Double> addPeriodic) {

    this.addPeriodic = addPeriodic;

    addPeriodic.accept(
        () -> {
          CANBusStatus status = canivoreCanbus.getStatus();
          DogLog.log("Canivore/Canivore Bus Utilization", status.BusUtilization);
          DogLog.log("Canivore/Status Code on Canivore", status.Status.toString());
        },
        0.5);

    // objDecCam =
    //     new ObjectDetectionCam(
    //         "cam2026_01", ObjectDetectionConstants.robotToCam, () ->
    // drivetrain.getCachedState().Pose);

    configureBindings();
    configureAutonomous();

    drivetrain.setDefaultCommand(defualtDriveCommand);

    CommandScheduler.getInstance()
        .schedule(
            Commands.parallel(
                    FollowPathCommand.warmupCommand(),
                    drivetrain.setSlowMode(false),
                    shooter.stopShooter(),
                    indexer.runVoltage(0),
                    groundIntakeExtension.retractFull(),
                    groundIntakeRoller.stopIntake())
                .withName("Warm Up Command")
                .ignoringDisable(true));

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    DogLog.log("Current Robot", getRobot().toString());

    if (RobotBase.isSimulation()) {
      configureFuelSim();
      SmartDashboard.putData(
          Commands.runOnce(
                  () -> {
                    FuelSim.getInstance().clearFuel();
                  })
              .withName("Reset Fuel")
              .ignoringDisable(true));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * org.wpilib.command2.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link org.wpilib.command2.button.CommandPS4Controller
   * PS4} controllers or {@link org.wpilib.command2.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    RobotModeTriggers.disabled().onTrue(disableHandler());
    controller.leftBumper().onTrue(drivetrain.setRotationCommand(RotationTarget.NORMAL));

    controller.rightTrigger().and(drivetrain.isInAllianceZone).whileTrue(shootHub());

    controller.rightTrigger().and(controller.povDown().negate()).whileTrue(agitateGroundIntake());

    controller.rightStick().whileTrue(agitateGroundIntake());
    controller.leftStick().whileTrue(agitateGroundIntake());

    controller
        .rightTrigger()
        .and(
            drivetrain
                .isInNeutralZone
                .or(drivetrain.isInOpponentAllianceZone)
                .and(drivetrain.isOnDepotSide))
        .whileTrue(shootDepot());
    controller
        .rightTrigger()
        .and(
            drivetrain
                .isInNeutralZone
                .or(drivetrain.isInOpponentAllianceZone)
                .and(drivetrain.isOnOutpostSide))
        .whileTrue(shootOutpost());
    controller.rightTrigger().onFalse(stopShoot());

    controller.a().onTrue(retractGroundIntake());

    controller.b().onTrue(blocker.retract());

    controller.x().whileTrue(defenseMode());

    controller.y().onTrue(blocker.deploy());

    drivetrain.isInAllianceZone.onTrue(shooter.preSpin());

    controller
        .rightBumper()
        .onTrue(drivetrain.setSlowMode(true))
        .onFalse(drivetrain.setSlowMode(false));

    controller.povDown().and(RobotModeTriggers.disabled().negate()).whileTrue(deployGroundIntake());
    controller.povDown().onFalse(groundIntakeRoller.stopIntake());
    // controller.povDown().onFalse(shooter.stopShooter().onlyIf(controller.rightTrigger().and(controller.povDown()).negate()));
    controller.povDown().and(controller.rightTrigger().negate()).onTrue(topoff());
    controller
        .povDown()
        .negate()
        .and(controller.rightTrigger().negate())
        .onTrue(shooter.stopShooter());

    controller
        .povUp()
        .and(RobotModeTriggers.disabled().negate())
        .whileTrue(deployDirectionalGroundIntake());
    controller.povUp().onFalse(groundIntakeRoller.stopIntake());
    controller
        .povUp()
        .onFalse(drivetrain.setRotationCommand(SwerveSubsystem.RotationTarget.NORMAL));

    controller
        .povUp()
        .negate()
        .and(controller.rightTrigger().negate())
        .onTrue(shooter.stopShooter());

    controller.povRight().whileTrue(bumpJump());
    controller.povRight().onFalse(stopBumpJump());

    controller.povLeft().whileTrue(declogShimmy());
    controller.povLeft().onFalse(drivetrain.setRotationCommand(RotationTarget.NORMAL));

    // temp
    controller.rightStick().whileTrue(backupShootHub());
    controller.rightStick().onFalse(stopShoot());
    controller.leftStick().whileTrue(backupShootTrench());
    controller.leftStick().onFalse(stopShoot());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureAutonomous() {
    NeutralAutos.configNeutralAutos(
        drivetrain, shooter, indexer, groundIntakeExtension, groundIntakeRoller, climber);
    autoChooser.addOption(
        "Bump 2 Cycle Depot", new NeutralAutos(false, Routine.BUMP, true, false, 0.0));
    autoChooser.addOption(
        "Bump 2 Cycle Outpost", new NeutralAutos(true, Routine.BUMP, true, false, 0.0));
    autoChooser.addOption(
        "Depot 1 Cycle",
        new DepotPathAuto_1c(
            drivetrain, shooter, indexer, groundIntakeExtension, groundIntakeRoller, false));
    autoChooser.addOption(
        "Depot Neutral",
        new DepotPathAuto_1c(
            drivetrain, shooter, indexer, groundIntakeExtension, groundIntakeRoller, true));
    autoChooser.setDefaultOption("Preload", new Preload(drivetrain, shooter, indexer));
    autoChooser.addOption(
        "4 Poofs 2nd pick Depot", new NeutralAutos(false, Routine.BUMP, true, true, 4.0));
    autoChooser.addOption(
        "4 Poofs 2nd pick Outpost", new NeutralAutos(true, Routine.BUMP, true, true, 4.0));
    autoChooser.addOption(
        "Super Intake Delay",
        new DelayAuto(shooter, indexer, groundIntakeExtension, groundIntakeRoller));
    SmartDashboard.putData("autonomous", autoChooser);
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
        0.660, // from left to right
        0.711, // from front to back
        0.127, // from floor to top of bumpers
        () -> drivetrain.getCachedState().Pose, // Supplier<Pose2d> of robot pose
        () ->
            drivetrain.getCachedState().Velocity.toFieldRelative(
                drivetrain.getCachedState().Pose.getRotation()));
    // Supplier<ChassisSpeeds> of field-centric chassis speeds

    // Register an intake to remove fuel from the field as a rectangular bounding box
    instance.registerIntake(
        0.350,
        0.700,
        -0.330,
        0.330,
        () ->
            (groundIntakeRoller.getGoalRollerVoltage()
                > 0)); // robot-centric coordinates for bounding box

    instance.start();
  }

  public void periodic() {

    boolean autoWin = false;
    if (HubTracker.getAutoWinner().isPresent()) {
      Alliance actualAutoWinner = HubTracker.getAutoWinner().get();
      if (EagleUtil.isRedAlliance()) {
        autoWin = actualAutoWinner == Alliance.RED;
      } else {
        autoWin = actualAutoWinner == Alliance.BLUE;
      }
    }

    DogLog.log("Auto Winner", autoWin);

    for(AprilTagCam cam : cameras) {
      cam.updatePoseEstim();
    }


    // if (objDecCam != null) {
    //   objDecCam.updateDetection();
    // }

    // DogLog.log(
    //     "Loop Time/Robot Container/objectDetection Cam",
    //     (RobotController.getTime() - startTime) / 1000);

    long startTime = RobotController.getTime();

    robotVisualizer.periodic();
    DogLog.log(
        "Loop Time/Robot Container/Robot Visualizer", (RobotController.getTime() - startTime) / 1000);

    startTime = RobotController.getTime();

    DogLog.log("Match Timer", MatchState.getMatchTime());

    DogLog.log("Loop Time/Robot Container/Match Timer", (RobotController.getTime() - startTime) / 1000);

    startTime = RobotController.getTime();

    Optional<Pose2d> obj = GamePieceTracker.getGamePiece();

    DogLog.log(
        "Loop Time/Robot Container/Game Piece Tracker", (RobotController.getTime() - startTime) / 1000);

    DogLog.log("Hub Status/Is Active", isHubActive.getAsBoolean());

    if (obj.isPresent()) {
      DogLog.log("Object Detection/Fuel Pose", new Pose2d[] {obj.get()}); // ill forget it tommorow
    } else {
      DogLog.log("Object Detection/Fuel Pose", new Pose2d[0]); // ill forget it tommorow
    }
  }

  private Command disableHandler() {
    return Commands.sequence(
            shooter.stopShooter(),
            drivetrain.setRotationCommand(RotationTarget.NORMAL),
            climber.runVoltage(0))
        .ignoringDisable(true)
        .withName("Disabled");
  }

  public Command shootHub() {
    return Commands.sequence(
            drivetrain.setRotationCommand(RotationTarget.HUB),
            drivetrain.setSlowMode(0.5, 1),
            Commands.waitUntil(
                drivetrain
                    .isFacingGoal
                    .debounce(0.25)
                    .and(isHubActive)
                    .or(controller.leftTrigger())),
            Commands.parallel(
                    indexer.index(), shooter.cruiseControl(), EagleUtil.shootInSim(drivetrain))
                .onlyWhile(drivetrain.isFacingGoal.and(isHubActive).or(controller.leftTrigger()))
                .repeatedly())
        .withName("Shoot Hub");
  }

  public Command shootDepot() {
    return Commands.parallel(
            drivetrain.setRotationCommand(RotationTarget.PASSING_DEPOT_SIDE),
            drivetrain.setSlowMode(0.5, 1),
            Commands.waitUntil(
                drivetrain.isFacingGoal.and(isHubActive).or(controller.leftTrigger())),
            Commands.parallel(
                    indexer.index(), shooter.cruiseControl(), EagleUtil.shootInSim(drivetrain))
                .onlyWhile(drivetrain.isFacingGoalPassing.or(controller.leftTrigger()))
                .repeatedly())
        .withName("Shoot Depot Side");
  }

  public Command shootOutpost() {
    return Commands.parallel(
            drivetrain.setRotationCommand(RotationTarget.PASSING_OUTPOST_SIDE),
            drivetrain.setSlowMode(0.5, 1),
            Commands.waitUntil(
                drivetrain.isFacingGoal.and(isHubActive).or(controller.leftTrigger())),
            Commands.parallel(
                    indexer.index(), shooter.cruiseControl(), EagleUtil.shootInSim(drivetrain))
                .onlyWhile(drivetrain.isFacingGoalPassing.or(controller.leftTrigger()))
                .repeatedly())
        .withName("Shoot Outpost Side");
  }

  public Command unStuck() {
    return Commands.parallel(
            indexer.reverse(), groundIntakeRoller.reverseIntake(), groundIntakeExtension.extend())
        .withName("Unjam");
  }

  public Command deployGroundIntake() {
    return Commands.parallel(
            groundIntakeRoller.startIntake(),
            groundIntakeExtension.extend(),
            drivetrain.temporarilyDisableRotation().onlyWhile(controller.rightTrigger().negate()))
        .withName("Deploy Ground Intake");
  }

  public Command deployDirectionalGroundIntake() {
    return Commands.parallel(
            groundIntakeRoller.startIntake(),
            groundIntakeExtension.extend(),
            drivetrain.setRotationCommand(RotationTarget.DIRECTION))
        .withName("Deploy Ground Intake");
  }

  public Command retractGroundIntake() {
    return Commands.parallel(
            groundIntakeRoller.stopIntake(),
            groundIntakeExtension.retractFull(),
            drivetrain.temporarilyDisableRotation().onlyWhile(controller.rightTrigger().negate()))
        .withName("Retract Ground Intake");
  }

  public Command defenseMode() {
    return Commands.parallel(drivetrain.swerveX()).withName("Defense Mode");
  }

  public Command agitateGroundIntake() {
    return Commands.sequence(
            groundIntakeRoller.stopIntake(),
            groundIntakeExtension.extend2().withTimeout(0.04),
            Commands.waitSeconds(.6),
            groundIntakeExtension.retract().withTimeout(0.04),
            Commands.waitSeconds(.6))
        .repeatedly()
        .withName("Agitate Ground Intake");
  }

  public Command stopShoot() {
    return Commands.parallel(
            drivetrain.setRotationCommand(RotationTarget.NORMAL),
            drivetrain.setSlowMode(false),
            shooter.stopShooter())
        .withName("Stop Shooting");
  }

  public Command topoff() {
    return Commands.parallel(
            indexer.runVoltage(0),
            groundIntakeExtension.extend(),
            groundIntakeRoller.startIntake(),
            shooter.runVelocity(0, 0))
        .withName("Topoff");
  }

  public Command backupShootHub() {
    return Commands.parallel(
            shooter.runVelocity(45, 42),
            indexer.index(),
            drivetrain.setRotationCommand(RotationTarget.NORMAL),
            EagleUtil.shootInSim(drivetrain))
        .withName("Shoot Hub Backup");
  }

  public Command backupShootTrench() {
    return Commands.parallel(
            shooter.runVelocity(68, 66),
            indexer.index(),
            drivetrain.setRotationCommand(RotationTarget.NORMAL),
            EagleUtil.shootInSim(drivetrain))
        .withName("Shoot Trench Backup");
  }

  public Command bumpJump() {
    return Commands.parallel(
        drivetrain.setRotationCommand(RotationTarget.FORTY_FIVE), drivetrain.setBumpSpeed(true));
  }

  public Command stopBumpJump() {
    return Commands.parallel(
        drivetrain.setRotationCommand(RotationTarget.NORMAL), drivetrain.setBumpSpeed(false));
  }

  public Command declogShimmy() {
    return Commands.sequence(
            drivetrain.setRotationCommand(RotationTarget.LEFT),
            Commands.waitSeconds(0.2),
            drivetrain.setRotationCommand(RotationTarget.RIGHT),
            Commands.waitSeconds(0.2))
        .repeatedly();
  }
}
