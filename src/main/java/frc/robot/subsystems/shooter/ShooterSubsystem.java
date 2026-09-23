package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.Alert;

import frc.robot.EagleUtil;
import frc.robot.RobotContainer;
import frc.robot.ShotCalculator;

import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX motor1; // Right Front
  private final TalonFX motor2; // Right Back
  private final TalonFX motor3; // Middle Front
  private final TalonFX motor4; // Middle Back
  private final TalonFX motor5; // Left Front
  private final TalonFX motor6; // Left Back

  private final VelocityVoltage velocityRequest1 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest2 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest3 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest4 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest5 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest6 = new VelocityVoltage(0).withEnableFOC(true);

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<AngularVelocity> motor1Velocity;
  private final StatusSignal<Temperature> motor1Temp;
  private final StatusSignal<Double> motor1ClosedLoopGoal;

  private final StatusSignal<Voltage> motor2Voltage;
  private final StatusSignal<Current> motor2StatorCurrent;
  private final StatusSignal<AngularVelocity> motor2Velocity;
  private final StatusSignal<Temperature> motor2Temp;
  private final StatusSignal<Double> motor2ClosedLoopGoal;

  private final StatusSignal<Voltage> motor3Voltage;
  private final StatusSignal<Current> motor3StatorCurrent;
  private final StatusSignal<AngularVelocity> motor3Velocity;
  private final StatusSignal<Temperature> motor3Temp;
  private final StatusSignal<Double> motor3ClosedLoopGoal;

  private final StatusSignal<Voltage> motor4Voltage;
  private final StatusSignal<Current> motor4StatorCurrent;
  private final StatusSignal<AngularVelocity> motor4Velocity;
  private final StatusSignal<Temperature> motor4Temp;
  private final StatusSignal<Double> motor4ClosedLoopGoal;

  private final StatusSignal<Voltage> motor5Voltage;
  private final StatusSignal<Current> motor5StatorCurrent;
  private final StatusSignal<AngularVelocity> motor5Velocity;
  private final StatusSignal<Temperature> motor5Temp;
  private final StatusSignal<Double> motor5ClosedLoopGoal;

  private final StatusSignal<Voltage> motor6Voltage;
  private final StatusSignal<Current> motor6StatorCurrent;
  private final StatusSignal<AngularVelocity> motor6Velocity;
  private final StatusSignal<Temperature> motor6Temp;
  private final StatusSignal<Double> motor6ClosedLoopGoal;

  private boolean isLeftEnabled = true;
  private boolean isRightEnabled = true;

  private final Alert motor1NotConnectedAlert =
      new Alert("Shooter Motor 1 Not Connected", Alert.Level.HIGH);
  private final Alert motor2NotConnectedAlert =
      new Alert("Shooter Motor 2 Not Connected", Alert.Level.HIGH);
  private final Alert motor3NotConnectedAlert =
      new Alert("Shooter Motor 3 Not Connected", Alert.Level.HIGH);
  private final Alert motor4NotConnectedAlert =
      new Alert("Shooter Motor 4 Not Connected", Alert.Level.HIGH);
  private final Alert motor5NotConnectedAlert =
      new Alert("Shooter Motor 5 Not Connected", Alert.Level.HIGH);
  private final Alert motor6NotConnectedAlert =
      new Alert("Shooter Motor 6 Not Connected", Alert.Level.HIGH);

  private double velocityGoal;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Pose2d> robotTargetSupplier;

  public final Trigger isAtGoalVelocity_Passing =
      new Trigger(() -> MathUtil.isNear(velocityGoal, getVelocity(), 10));
  public final Trigger isAtGoalVelocity_Hub =
      new Trigger(() -> MathUtil.isNear(velocityGoal, getVelocity(), 5));

  public ShooterSubsystem(
      CANBus canBus, Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget) {
    
    switch(RobotContainer.getRobot()) {
      case KITBOT:
        motor1 = new TalonFX(ShooterConstants.MOTOR_1_ID, canBus);
        motor2 = TalonFX.none();
        motor3 = TalonFX.none();
        motor4 = TalonFX.none();
        motor5 = TalonFX.none();
        motor6 = TalonFX.none();
        break;
      case ANEMONE:
      case DEV:
        motor1 = TalonFX.none();
        motor2 = TalonFX.none();
        motor3 = TalonFX.none();
        motor4 = TalonFX.none();
        motor5 = TalonFX.none();
        motor6 = TalonFX.none();
        break;
      case SIM:
      case COMP:
      default:
        motor1 = new TalonFX(ShooterConstants.MOTOR_1_ID, canBus);
        motor2 = new TalonFX(ShooterConstants.MOTOR_2_ID, canBus);
        motor3 = new TalonFX(ShooterConstants.MOTOR_3_ID, canBus);
        motor4 = new TalonFX(ShooterConstants.MOTOR_4_ID, canBus);
        motor5 = new TalonFX(ShooterConstants.MOTOR_5_ID, canBus);
        motor6 = new TalonFX(ShooterConstants.MOTOR_6_ID, canBus);
        break;
    }

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Velocity = motor1.getVelocity();
    motor1Temp = motor1.getDeviceTemp();
    motor1ClosedLoopGoal = motor1.getClosedLoopReference();

    motor2Voltage = motor2.getMotorVoltage();
    motor2StatorCurrent = motor2.getStatorCurrent();
    motor2Velocity = motor2.getVelocity();
    motor2Temp = motor2.getDeviceTemp();
    motor2ClosedLoopGoal = motor2.getClosedLoopReference();

    motor3Voltage = motor3.getMotorVoltage();
    motor3StatorCurrent = motor3.getStatorCurrent();
    motor3Velocity = motor3.getVelocity();
    motor3Temp = motor3.getDeviceTemp();
    motor3ClosedLoopGoal = motor3.getClosedLoopReference();

    motor4Voltage = motor4.getMotorVoltage();
    motor4StatorCurrent = motor4.getStatorCurrent();
    motor4Velocity = motor4.getVelocity();
    motor4Temp = motor4.getDeviceTemp();
    motor4ClosedLoopGoal = motor4.getClosedLoopReference();

    motor5Voltage = motor5.getMotorVoltage();
    motor5StatorCurrent = motor5.getStatorCurrent();
    motor5Velocity = motor5.getVelocity();
    motor5Temp = motor5.getDeviceTemp();
    motor5ClosedLoopGoal = motor5.getClosedLoopReference();

    motor6Voltage = motor6.getMotorVoltage();
    motor6StatorCurrent = motor6.getStatorCurrent();
    motor6Velocity = motor6.getVelocity();
    motor6Temp = motor6.getDeviceTemp();
    motor6ClosedLoopGoal = motor6.getClosedLoopReference();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor1Voltage,
        motor1StatorCurrent,
        motor1Velocity,
        motor1Temp,
        motor1ClosedLoopGoal,
        motor2Voltage,
        motor2StatorCurrent,
        motor2Velocity,
        motor2Temp,
        motor2ClosedLoopGoal,
        motor3Voltage,
        motor3StatorCurrent,
        motor3Velocity,
        motor3Temp,
        motor3ClosedLoopGoal,
        motor4Voltage,
        motor4StatorCurrent,
        motor4Velocity,
        motor4Temp,
        motor4ClosedLoopGoal,
        motor5Voltage,
        motor5StatorCurrent,
        motor5Velocity,
        motor5Temp,
        motor5ClosedLoopGoal,
        motor6Voltage,
        motor6StatorCurrent,
        motor6Velocity,
        motor6Temp,
        motor6ClosedLoopGoal);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Slot0.kS = 0.2;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.122;
    talonFXConfig.Slot0.kP = 0.1;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor1.getConfigurator().apply(talonFXConfig);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor2.getConfigurator().apply(talonFXConfig);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor3.getConfigurator().apply(talonFXConfig);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor4.getConfigurator().apply(talonFXConfig);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor5.getConfigurator().apply(talonFXConfig);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor6.getConfigurator().apply(talonFXConfig);

    robotTargetSupplier = robotTarget;
    robotPoseSupplier = robotPose;
    SmartDashboard.putData("Alternate Shooter", alternateLeftRight());
  }

  public Command runVelocity(double rotationsPerSecond) {
    return this.run(
            () -> {
              runShooterWithClamp(rotationsPerSecond, rotationsPerSecond);
            })
        .withName("Run Velocity");
  }

  public Command runVelocity(double frontrps, double backrps) {
    return this.run(
            () -> {
              runShooterWithClamp(frontrps, backrps);
            })
        .withName("Run Velocity");
  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
            () -> {
              if (isRightEnabled) {
      motor1.setVoltage(voltage);
      motor2.setVoltage(voltage);
    }
    motor3.setVoltage(voltage);
    motor4.setVoltage(voltage);
    if (isLeftEnabled) {
      motor5.setVoltage(voltage);
      motor6.setVoltage(voltage);
    }
            })
        .withName("Run Voltage");
  }

  public Command cruiseControl() {
    return this.run(
            () -> {
              Pose2d robotPose = robotPoseSupplier.get();
              Pose2d targetPose = robotTargetSupplier.get();
              double robotTargetDist = EagleUtil.getRobotTargetDistance(robotPose, targetPose);
              double frontRotationsPerSecond = ShotCalculator.getFrontVelocity(robotTargetDist);
              double backRotationsPerSecond = ShotCalculator.getBackVelocity(robotTargetDist);

              runShooterWithClamp(frontRotationsPerSecond, backRotationsPerSecond);
            })
        .withName("Cruise Control");
  }

  private void runShooterWithClamp(double frontrps, double backrps) {
    double clampedFrontRps =
        Math.max(ShooterConstants.MIN_RPS, Math.min(ShooterConstants.MAX_RPS, frontrps));
    double clampedBackRps =
        Math.max(ShooterConstants.MIN_RPS, Math.min(ShooterConstants.MAX_RPS, backrps));
    velocityGoal = clampedFrontRps;

    if (isRightEnabled) {
      motor1.setControl(velocityRequest1.withVelocity(clampedFrontRps));
      motor2.setControl(velocityRequest2.withVelocity(clampedBackRps));
    }
    motor3.setControl(velocityRequest3.withVelocity(clampedFrontRps));
    motor4.setControl(velocityRequest4.withVelocity(clampedBackRps));
    if (isLeftEnabled) {
      motor5.setControl(velocityRequest5.withVelocity(clampedFrontRps));
      motor6.setControl(velocityRequest6.withVelocity(clampedBackRps));
    }
  }

  public Command preSpin() {
    // return this.run(
    //         () -> {
    //           Pose2d robotPose = robotPoseSupplier.get();
    //           Pose2d targetPose = robotTargetSupplier.get();
    //           double robotTargetDist = EagleUtil.getRobotTargetDistance(robotPose, targetPose);
    //           double frontRotationsPerSecond = ShotCalculator.getFrontVelocity(robotTargetDist);
    //           double backRotationsPerSecond = ShotCalculator.getBackVelocity(robotTargetDist);
    //           runVoltage(0);

    //           // does not actually pre-spin
    //         })
    //     .withName("Pre Spin");
    return Commands.none();
  }

  public Command setLeftShooterEnabled(boolean enable) {
    return Commands.runOnce(
        () -> {
          isLeftEnabled = enable;
        });
  }

  public Command setRightShooterEnabled(boolean enable) {
    return Commands.runOnce(
        () -> {
          isRightEnabled = enable;
        });
  }

  public Command alternateLeftRight() {
    return Commands.sequence(
            setLeftShooterEnabled(true),
            setRightShooterEnabled(false),
            Commands.waitSeconds(5),
            setLeftShooterEnabled(false),
            setRightShooterEnabled(true),
            Commands.waitSeconds(5))
        .repeatedly()
        .withName("Alternate turning on/off right and left shooter");
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(motor1Voltage,
        motor1StatorCurrent,
        motor1Velocity,
        motor1Temp,
        motor1ClosedLoopGoal,
        motor2Voltage,
        motor2StatorCurrent,
        motor2Velocity,
        motor2Temp,
        motor2ClosedLoopGoal,
        motor3Voltage,
        motor3StatorCurrent,
        motor3Velocity,
        motor3Temp,
        motor3ClosedLoopGoal,
        motor4Voltage,
        motor4StatorCurrent,
        motor4Velocity,
        motor4Temp,
        motor4ClosedLoopGoal,
        motor5Voltage,
        motor5StatorCurrent,
        motor5Velocity,
        motor5Temp,
        motor5ClosedLoopGoal,
        motor6Voltage,
        motor6StatorCurrent,
        motor6Velocity,
        motor6Temp,
        motor6ClosedLoopGoal);

    DogLog.log("Shooter/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 1 Velocity", motor1Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 1 Closed Loop Goal", motor1ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 2 Voltage", motor2Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 2 Stator Current", motor2StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 2 Velocity", motor2Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 2 Temperature", motor2Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 2 Closed Loop Goal", motor2ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 3 Voltage", motor3Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 3 Stator Current", motor3StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 3 Velocity", motor3Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 3 Temperature", motor3Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 3 Closed Loop Goal", motor3ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 4 Voltage", motor4Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 4 Stator Current", motor4StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 4 Velocity", motor4Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 4 Temperature", motor4Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 4 Closed Loop Goal", motor4ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 5 Voltage", motor5Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 5 Stator Current", motor5StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 5 Velocity", motor5Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 5 Temperature", motor5Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 5 Closed Loop Goal", motor5ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 6 Voltage", motor6Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 6 Stator Current", motor6StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 6 Velocity", motor6Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 6 Temperature", motor6Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 6 Closed Loop Goal", motor6ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Left Shooter Enabled", isLeftEnabled);
    DogLog.log("Shooter/Right Shooter Enabled", isRightEnabled);

    DogLog.log(
        "Shooter/Motor 1 and 2 Velocity Difference",
        motor1Velocity.getValueAsDouble() - motor2Velocity.getValueAsDouble());
    DogLog.log(
        "Shooter/Motor 3 and 4 Velocity Difference",
        motor3Velocity.getValueAsDouble() - motor4Velocity.getValueAsDouble());
    DogLog.log(
        "Shooter/Motor 5 and 6 Velocity Difference",
        motor5Velocity.getValueAsDouble() - motor6Velocity.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
    motor2NotConnectedAlert.set(!motor2.isConnected());
    motor3NotConnectedAlert.set(!motor3.isConnected());
    motor4NotConnectedAlert.set(!motor4.isConnected());
    motor5NotConnectedAlert.set(!motor5.isConnected());
    motor6NotConnectedAlert.set(!motor6.isConnected());

    if (!isLeftEnabled) {
      motor5.setVoltage(0);
      motor6.setVoltage(0);
    }
    if (!isRightEnabled) {
      motor1.setVoltage(0);
      motor2.setVoltage(0);
    }
  

    DogLog.log("Shooter/ Velocity", getVelocity());
    DogLog.log("Shooter/Goal Velocity", velocityGoal);
    DogLog.log("Shooter/At Goal Velocity Hub", this.isAtGoalVelocity_Hub.getAsBoolean());
    DogLog.log("Shooter/At Goal Velocity Passing", this.isAtGoalVelocity_Passing.getAsBoolean());
  }

  /**
   * @return the shooter's rps
   */
  public double getVelocity() {
    return (motor1Velocity.getValueAsDouble()
            + motor2Velocity.getValueAsDouble()
            + motor3Velocity.getValueAsDouble()
            + motor4Velocity.getValueAsDouble()
            + motor5Velocity.getValueAsDouble()
            + motor6Velocity.getValueAsDouble())
        / 6;
  }

  public Command stopShooter() {
    return this.runOnce(
            () -> {
              motor1.stopMotor();
              motor2.stopMotor();
              motor3.stopMotor();
              motor4.stopMotor();
              motor5.stopMotor();
              motor6.stopMotor();
              velocityGoal = 0.0;
            })
        .withName("Stop Shooter");
  }
}
