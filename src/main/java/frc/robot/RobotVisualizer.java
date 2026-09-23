package frc.robot;

import dev.doglog.DogLog;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.command2.SubsystemBase;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionConstants;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;

public class RobotVisualizer extends SubsystemBase {
  private final GroundIntakeLinearExtensionSubsystem linearExt;

  public RobotVisualizer(GroundIntakeLinearExtensionSubsystem linearExt) {
    this.linearExt = linearExt;
  }

  @Override
  public void periodic() {
    double groundIntakeExtension =
        linearExt.getRotation()
            * 0.35
            / GroundIntakeLinearExtensionConstants.EXTENSION_ROTATION; // 35cm full out

    Pose3d groundIntakeRack = new Pose3d(groundIntakeExtension, 0, 0, new Rotation3d());
    Pose3d groundIntakePosition =
        new Pose3d(
            groundIntakeExtension + 0.1375,
            0,
            0.198,
            new Rotation3d(0, groundIntakeExtension / 0.35 * Math.PI / 2, 0)); // pi/2 full rotation

    DogLog.log(
        "Robot Visualizer/Component Positions",
        new Pose3d[] {groundIntakeRack, groundIntakePosition});

    DogLog.log("get rotation", groundIntakeExtension);
  }
}
