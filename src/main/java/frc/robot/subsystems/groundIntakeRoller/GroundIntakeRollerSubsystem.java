// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntakeRoller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import frc.robot.RobotContainer;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alert;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class GroundIntakeRollerSubsystem extends SubsystemBase {

    private double groundRollerVoltage;

  private final TalonFX motor1;
  private final TalonFX motor2;

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<Temperature> motor1Temp;

  private final StatusSignal<Voltage> motor2Voltage;
  private final StatusSignal<Current> motor2StatorCurrent;
  private final StatusSignal<Temperature> motor2Temp;


  private final Alert motor1NotConnectedAlert =
      new Alert("Ground Intake Roller Motor 1 Not Connected ", Alert.Level.HIGH);

  private final Alert motor2NotConnectedAlert =
      new Alert("Ground Intake Roller Motor 2 Not Connected ", Alert.Level.HIGH);

  public GroundIntakeRollerSubsystem(CANBus canBus) {
    switch(RobotContainer.getRobot()) {
      case KITBOT:
      case ANEMONE:
      case DEV:
        motor1 = TalonFX.none();
        motor2 = TalonFX.none();
        break;
      case SIM:
      case COMP:
      default:
        motor1 = new TalonFX(GroundIntakeRollerConstants.MOTOR_1_ID, canBus);
        motor2 = new TalonFX(GroundIntakeRollerConstants.MOTOR_2_ID, canBus);
        break;
    }


    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Temp = motor1.getDeviceTemp();

    motor2Voltage = motor2.getMotorVoltage();
    motor2StatorCurrent = motor2.getStatorCurrent();
    motor2Temp = motor2.getDeviceTemp();

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 90;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor1.getConfigurator().apply(talonFXConfig);
    motor2.getConfigurator().apply(talonFXConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, motor1StatorCurrent, motor1Temp, motor2StatorCurrent, motor2Temp);

    BaseStatusSignal.setUpdateFrequencyForAll(250, motor1Voltage, motor2Voltage);
  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
        () -> {
          motor1.setVoltage(-voltage);
          motor2.setVoltage(voltage);
          groundRollerVoltage = voltage;
        });
  }

  public Command stopIntake() {
    return runVoltage(0);
  }

  public Command startIntake() {
    return runVoltage(GroundIntakeRollerConstants.INTAKE_VOLTAGE);
  }

  public Command reverseIntake() {
    return runVoltage(GroundIntakeRollerConstants.REVERSE_INTAKE_VOLTAGE);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(motor1Voltage, motor1StatorCurrent, motor1Temp, motor2Voltage, motor2StatorCurrent, motor2Temp);

    DogLog.log("Ground Roller/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 2 Voltage", motor2Voltage.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 2 Stator Current", motor2StatorCurrent.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 2 Temperature", motor2Temp.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
    motor2NotConnectedAlert.set(!motor2.isConnected());
  }

  public double getGoalRollerVoltage() {
    return groundRollerVoltage;
  }
}
