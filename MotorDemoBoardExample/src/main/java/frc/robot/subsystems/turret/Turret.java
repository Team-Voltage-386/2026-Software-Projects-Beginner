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

  private final Supplier<Pose2d> dtPose;
  private Pose3d currentTargetPose = new Pose3d();
  private final Flywheel flywheel;

  private boolean autoAimEnabled = false;

  public Turret(TurretIO io, Supplier<Pose2d> dtPose, Flywheel flywheel) {
    this.io = io;
    this.dtPose = dtPose;
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret/Inputs", inputs);

    Pose2d pose =
        dtPose.get().plus(new Transform2d(TurretConstants.turretPosition, Rotation2d.kZero));
    Translation3d dtPos3d =
        new Translation3d(pose.getTranslation().getX(), pose.getTranslation().getY(), 0.5);
    turretVisual[0] = new Pose3d(dtPos3d, new Rotation3d());
    turretVisual[1] =
        new Pose3d(
            dtPos3d.plus(
                new Translation3d(
                    inputs.turretYaw.plus(pose.getRotation()).getCos() * 0.5,
                    inputs.turretYaw.plus(pose.getRotation()).getSin() * 0.5,
                    // TODO: this isn't accurate for high pitch angles
                    inputs.turretPitch.getSin() * 0.65)),
            new Rotation3d(dtPose.get().getRotation()));

    Logger.recordOutput("Shooter/Turret/Visual", turretVisual);
  }
}
