// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkFlex;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSparkMax;
import frc.robot.util.TuningUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Subsystems
  private Flywheel flywheel;
  private Turret turret;

  private double flywheelSpeed = 0;

  TuningUtil setRPM = new TuningUtil("/Tuning/Flywheel/TestSetRPM", 0);
  TuningUtil setDegrees = new TuningUtil("/Tuning/Turret/TestSetDegrees", 62);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        flywheel = new Flywheel(new FlywheelIOSparkFlex());
        turret = new Turret(new TurretIOSparkMax(), flywheel);

        break;

      case SIM:
        flywheel = new Flywheel(new FlywheelIOSim());
        turret = null;
        break;

      case REPLAY:
        break;

      default:
        break;
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controller
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> flywheel.setFlywheelSpeed(setRPM.getValue())));
    controller.rightTrigger().onFalse(new InstantCommand(() -> flywheel.setFlywheelSpeed(0)));

    controller.a().onTrue(new InstantCommand(() -> flywheel.setFlywheelSpeed(setRPM.getValue())));
    controller.b().onTrue(new InstantCommand(() -> flywheel.setFlywheelSpeed(0)));
    controller.x().onTrue(turret.runOnce(() -> ((TurretIOSparkMax) turret.io).setHoodZero()));

    controller.povUp().onTrue(turret.manualIncrementPitch(Rotation2d.fromDegrees(-1)));
    controller.povDown().onTrue(turret.manualIncrementPitch(Rotation2d.fromDegrees(1)));

    controller.leftTrigger().whileTrue(turret.adjustPitch(() -> setDegrees.getValue()));
    controller.leftTrigger().onFalse(turret.adjustPitch(() -> 62.0));
    controller
        .povLeft()
        .whileTrue(
            new FunctionalCommand(
                () -> {},
                () -> {
                  System.out.println("running at " + 1);
                  ((TurretIOSparkMax) turret.io).testHoodVoltage(1.0);
                },
                (c) -> {
                  ((TurretIOSparkMax) turret.io).testHoodVoltage(0);
                },
                () -> false,
                flywheel));
    controller
        .povRight()
        .whileTrue(
            new FunctionalCommand(
                () -> {},
                () -> {
                  System.out.println("running at " + 1);
                  ((TurretIOSparkMax) turret.io).testHoodVoltage(-1.0);
                },
                (c) -> {
                  ((TurretIOSparkMax) turret.io).testHoodVoltage(0);
                },
                () -> false,
                flywheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public void updateNTValues() {}
}
