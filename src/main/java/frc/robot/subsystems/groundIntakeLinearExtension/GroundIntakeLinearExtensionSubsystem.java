package frc.robot.subsystems.groundIntakeLinearExtension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alert;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class GroundIntakeLinearExtensionSubsystem extends SubsystemBase {

  private final TalonFX motor;

  private final CANcoder groundIntakePivotEncoder;

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<Double> motorClosedLoopGoal;
  private final StatusSignal<Angle> motorPosition;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

  private final Alert motorNotConnectedAlert =
      new Alert("Ground Intake Linear Extension Motor 1 Not Connected", Alert.Level.HIGH);
  private final Alert groundIntakePivotEncoderAlert =
      new Alert("Ground Intake Linear Extension Encoder Not Connected", Alert.Level.HIGH);
  

  public GroundIntakeLinearExtensionSubsystem(CANBus canBus) {
    motor = new TalonFX(GroundIntakeLinearExtensionConstants.MOTOR_ID, canBus);

    groundIntakePivotEncoder =
        new CANcoder(GroundIntakeLinearExtensionConstants.PIVOT_ENCODER_ID, canBus);

    motorVoltage = motor.getMotorVoltage();
    motorStatorCurrent = motor.getStatorCurrent();
    motorClosedLoopGoal = motor.getClosedLoopReference();
    motorPosition = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, motorVoltage, motorStatorCurrent, motorClosedLoopGoal, motorPosition);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 30;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Feedback.FeedbackRotorOffset = 0;
    talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonFXConfig.Feedback.FeedbackRemoteSensorID =
        GroundIntakeLinearExtensionConstants.PIVOT_ENCODER_ID;
    talonFXConfig.Feedback.SensorToMechanismRatio = 40.0 / 18.0;
    talonFXConfig.Feedback.RotorToSensorRatio = 42.0 / 12.0 * 42.0 / 38.0 * 62.0 / 18.0;

    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        GroundIntakeLinearExtensionConstants.MAX_ROTATION;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        GroundIntakeLinearExtensionConstants.MIN_ROTATION;

    talonFXConfig.MotionMagic.MotionMagicAcceleration =
        GroundIntakeLinearExtensionConstants.MAX_ACCELERATION;
    talonFXConfig.MotionMagic.MotionMagicCruiseVelocity =
        GroundIntakeLinearExtensionConstants.MAX_VELOCITY;

    talonFXConfig.Slot0.kP = 50;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.Slot0.kS = 0.125;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.1125 * 26.6;

    motor.getConfigurator().apply(talonFXConfig);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoderConfig.MagnetSensor.MagnetOffset = 1;

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    groundIntakePivotEncoder.getConfigurator().apply(encoderConfig);

    motor.setPosition(0);
  }

  @Override
  public void periodic() {
    DogLog.log("GroundIntakeLinearExtension/Motor 1 Voltage", motorVoltage.getValueAsDouble());
    DogLog.log(
        "GroundIntakeLinearExtension/Motor 1 Stator Current",
        motorStatorCurrent.getValueAsDouble());
    DogLog.log(
        "GroundIntakeLinearExtension/Motor 1 Closed Loop Goal",
        motorClosedLoopGoal.getValueAsDouble());
    DogLog.log("GroundIntakeLinearExtension/Motor 1 Position", motorPosition.getValueAsDouble());

    motorNotConnectedAlert.set(!motor.isConnected());
    groundIntakePivotEncoderAlert.set(!groundIntakePivotEncoder.isConnected());

    BaseStatusSignal.refreshAll(
        motorVoltage, motorStatorCurrent, motorClosedLoopGoal, motorPosition);
  }

  public Command extend() {
    return Commands.sequence(
        this.runOnce(
            () -> {
              motor.setControl(request.withPosition(
                  GroundIntakeLinearExtensionConstants.EXTENSION_ROTATION));
            }),
        Commands.waitSeconds(2));
  }

  public Command extend2() {
    return this.runOnce(
        () -> {
          motor.setControl(request.withPosition(
              GroundIntakeLinearExtensionConstants.EXTENSION_ROTATION));
        });
  }

  public Command retract() {
    return this.runOnce(
        () -> {
          motor.setControl(request.withPosition(
              GroundIntakeLinearExtensionConstants.RETRACT_ROTATION));
        });
  }

  public Command retractFull() {
    return this.runOnce(
        () -> {
          motor.setControl(request.withPosition(
              GroundIntakeLinearExtensionConstants.RETRACT_FULL_ROTATION));
        });
  }

  public double getRotation() {
    return motorPosition.getValueAsDouble();
  }

  /*
   * Simulation
   */
  private DCMotorSim motorSimModel =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(DCMotor.getFalcon500Foc(1), 0.03, 42.0 / 12.0 * 42.0 / 38.0 * 62.0 / 18.0),
          DCMotor.getFalcon500Foc(1),
          0.1,
          0.1);
  
  @Override
  public void simulationPeriodic() {
    var talonFXSim = motor.getSimState();

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
    talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition() * 42.0 / 12.0 * 42.0 / 38.0 * 62.0 / 18.0);
    talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity() * 42.0 / 12.0 * 42.0 / 38.0 * 62.0 / 18.0);
  }
}
