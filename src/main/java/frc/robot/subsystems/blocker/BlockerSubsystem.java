// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.blocker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import frc.robot.RobotContainer;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alert;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class BlockerSubsystem extends SubsystemBase {

    private final TalonFX motor1;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

  private final Alert motor1NotConnectedAlert =
      new Alert("Blocker Motor 1 Not Connected", Alert.Level.HIGH);

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<AngularAcceleration> motor1Acceleration;
  private final StatusSignal<Temperature> motor1Temp;
  private final StatusSignal<Double> motor1ClosedLoopGoal;
  private final StatusSignal<Angle> motor1Position;

  /** Creates a new blocker. */
  public BlockerSubsystem(CANBus canBus) {

    switch(RobotContainer.getRobot()) {
      case SIM:
        motor1 = new TalonFX(BlockerConstants.MOTOR_1_ID, canBus);
        break;
      case KITBOT:
      case ANEMONE:
      case DEV:
      case COMP:
      default:
        motor1 = TalonFX.none();
        break;
    }

    

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Temp = motor1.getDeviceTemp();
    motor1Acceleration = motor1.getAcceleration();
    motor1ClosedLoopGoal = motor1.getClosedLoopReference();
    motor1Position = motor1.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal,
        motor1Position);

    motor1NotConnectedAlert.set(!motor1.isConnected());

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 30;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Slot0.kS = 0;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.12;
    talonFXConfig.Slot0.kP = 20;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = BlockerConstants.MAX_ROTATION;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BlockerConstants.MIN_ROTATION;

    talonFXConfig.MotionMagic.MotionMagicAcceleration = 50;
    talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = 65;

    motor1.getConfigurator().apply(talonFXConfig);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal,
        motor1Position);

    DogLog.log("Blocker/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Acceleration", motor1Acceleration.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Closed Loop Goal", motor1ClosedLoopGoal.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Position", motor1Position.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
  }

  public double getMotor1Position() {
    return motor1Position.getValueAsDouble();
  }

  public Command deploy() {
    return this.runOnce(
        () -> {
          motor1.setControl(request.withPosition(BlockerConstants.MAX_ROTATION));
        });
  }

  public Command retract() {
    return this.runOnce(
        () -> {
          motor1.setControl(request.withPosition(BlockerConstants.MIN_ROTATION));
        });
  }

  /*
   * Simulation
   */
  private DCMotorSim motorSimModel =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(DCMotor.getFalcon500Foc(1), 0.03, 3),
          DCMotor.getFalcon500Foc(1),
          0.1,
          0.1);
  
  @Override
  public void simulationPeriodic() {
    var talonFXSim = motor1.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    double motorVoltage = talonFXSim.getMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage);
    motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition() * 3.0);
    talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity() * 3.0);
  }

}
