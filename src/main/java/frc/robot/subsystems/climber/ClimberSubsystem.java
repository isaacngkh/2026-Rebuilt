// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import dev.doglog.DogLog;
import frc.robot.RobotContainer;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.MathUtil;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alert;
import org.wpilib.framework.RobotBase;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX motor1;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

  private final VoltageOut voltageOut = new VoltageOut(0);

  private final Alert motor1NotConnectedAlert =
      new Alert("Climber Motor 1 Not Connected", Alert.Level.HIGH);

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<AngularAcceleration> motor1Acceleration;
  private final StatusSignal<Temperature> motor1Temp;
  private final StatusSignal<Double> motor1ClosedLoopGoal;
  private final StatusSignal<Angle> motor1Position;

  private final StatusSignal<ForwardLimitValue> forwardLimit;
  private final StatusSignal<ReverseLimitValue> reverseLimit;

  private double goalRotation;

  public ClimberSubsystem(CANBus canBus) {
    switch(RobotContainer.getRobot()) {
      case SIM:
        motor1 = new TalonFX(ClimberConstants.MOTOR_1_ID, canBus);
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
    forwardLimit = motor1.getForwardLimit();
    reverseLimit = motor1.getReverseLimit();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal,
        motor1Position);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 30;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Slot0.kS = 0.125;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.065;
    talonFXConfig.Slot0.kP = 0.5;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.MAX_ROTATION;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.MIN_ROTATION;

    talonFXConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    talonFXConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    talonFXConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;

    talonFXConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    talonFXConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    talonFXConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    motor1.getConfigurator().apply(talonFXConfig);
  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
        () -> {
          motor1.setVoltage(voltage);
        });
  }

  public Command runPosition(double rotation) {
    return this.runOnce(
            () -> {
              double rotationClamp =
                  Math.clamp(
                      rotation, ClimberConstants.MIN_ROTATION, ClimberConstants.MAX_ROTATION);
              motor1.setControl(request.withPosition(rotationClamp));
              goalRotation = rotationClamp;
            })
        .andThen(
            Commands.waitUntil(
                () -> MathUtil.isNear(goalRotation, getMotor1Position(), 0.1)));
  }

  private Command setPosition(double rotation) {
    return this.runOnce(
        () -> {
          motor1.setPosition(rotation);
        });
  }

  private Command runVoltage(double voltage, boolean ignoreSoftwareLimit) {
    return this.runOnce(
        () -> {
          motor1.setControl(voltageOut.withIgnoreSoftwareLimits(ignoreSoftwareLimit).withOutput(voltage));
        });
  }

  public boolean getReverseLimitSwitch() {
    return reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
  }

  public Command homingCommand() {
    return Commands.sequence(
            runVoltage(-3, true),
            Commands.waitUntil(() -> getReverseLimitSwitch()),
            runVoltage(0),
            setPosition(0))
        .onlyIf(() -> RobotBase.isReal());
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal,
        motor1Position,
        forwardLimit,
        reverseLimit);
        
    DogLog.log("Climber/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Acceleration", motor1Acceleration.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Closed Loop Goal", motor1ClosedLoopGoal.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Position", motor1Position.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
  }

  public double getMotor1Position() {
    return motor1Position.getValueAsDouble();
  }

  /*
   * Simulation
   */
  private DCMotorSim motorSimModel =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(DCMotor.getFalcon500Foc(1), 0.03, 100),
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
    talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition() * 100.0);
    talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity() * 100.0);
  }
}
