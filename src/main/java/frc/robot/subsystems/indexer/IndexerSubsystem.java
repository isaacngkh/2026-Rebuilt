// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

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
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alert;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class IndexerSubsystem extends SubsystemBase {

  private double indexerVoltage;

  private final TalonFX motor1;

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<Temperature> motor1Temp;

  private final Alert motor1NotConnectedAlert =
      new Alert("Indexer Motor 1 Not Connected ", Alert.Level.HIGH);  

  public IndexerSubsystem(CANBus canBus) {
    switch(RobotContainer.getRobot()) {
      case ANEMONE:
      case DEV:
        motor1 = TalonFX.none();
        break;
      case KITBOT:
      case SIM:
      case COMP:
      default:
        motor1 = new TalonFX(IndexerConstants.MOTOR_1_ID, canBus);
        break;
    }

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Temp = motor1.getDeviceTemp();

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 55;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor1.getConfigurator().apply(talonFXConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(50, motor1Voltage, motor1StatorCurrent, motor1Temp);
  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
        () -> {
          motor1.setVoltage(voltage);
          indexerVoltage = voltage;
        });
  }

  public Command index() {
    return Commands.sequence(runVoltage(IndexerConstants.INDEXING_VOLTAGE), Commands.idle())
        .finallyDo(
            () -> {
              motor1.setVoltage(0);
              indexerVoltage = 0;
            });
  }

  public Command reverse() {
    return Commands.sequence(runVoltage(IndexerConstants.UNJAM_VOLTAGE), Commands.idle())
        .finallyDo(
            () -> {
              motor1.setVoltage(0);
              indexerVoltage = 0;
            });
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(motor1Voltage, motor1StatorCurrent, motor1Temp);

    DogLog.log("Indexer/Goal Voltage", indexerVoltage);

    DogLog.log("Indexer/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Indexer/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Indexer/Motor 1 Temperature", motor1Temp.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
  }
}
