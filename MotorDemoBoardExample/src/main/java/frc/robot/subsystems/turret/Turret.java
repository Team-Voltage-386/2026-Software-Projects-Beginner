package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.jr.TurretConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Handles turret control and aiming functionality */
public class Turret extends SubsystemBase {

  public final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Pose3d[] turretVisual = new Pose3d[2];

  private Pose3d currentTargetPose = new Pose3d();
  private final Flywheel flywheel;

  private boolean autoAimEnabled = false;

  public Turret(TurretIO io, Flywheel flywheel) {
    this.io = io;
    this.flywheel = flywheel;

    io.setTurretPitch(Rotation2d.fromDegrees(62));
    io.setTurretYaw(Rotation2d.kZero);
  }

  public Command addPitchCommand(Rotation2d deltaPitch) {
    return runOnce(() -> io.setTurretPitch(inputs.turretPitch.plus(deltaPitch)));
  }

  public Command addYawCommand(Rotation2d deltaYaw) {
    return runOnce(() -> io.setTurretYaw(inputs.turretYaw.plus(deltaYaw)));
  }

  private Rotation2d manualPitch = new Rotation2d(TurretConstants.turretMaxHoodAngle);

  public Command enableAutoAimCommand(Supplier<Pose3d> targetPose) {
    return runOnce(
        () -> {
          autoAimEnabled = true;
          currentTargetPose = targetPose.get();
        });
  }

  public Command disableAutoAimCommand() {
    return runOnce(() -> autoAimEnabled = false);
  }

  public Command adjustPitch(Supplier<Double> pitchDeg) {
    return runOnce(() -> io.setTurretPitch(new Rotation2d(Math.toRadians(pitchDeg.get()))));
  }

  public Command manualIncrementPitch(Rotation2d deltaPitch) {
    return runOnce(
        () -> {
          manualPitch = manualPitch.plus(deltaPitch);
          if (manualPitch.getDegrees() > TurretConstants.turretMaxHoodAngle) {
            manualPitch = Rotation2d.fromDegrees(TurretConstants.turretMaxHoodAngle);
          } else if (manualPitch.getDegrees() < TurretConstants.turretMinHoodAngle) {
            manualPitch = Rotation2d.fromDegrees(TurretConstants.turretMinHoodAngle);
          }
          io.setTurretPitch(manualPitch);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret/Inputs", inputs);
  }
}
