// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.motors.*;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private Constants.MotorSelection motorSel = Constants.MotorSelection.NEO1;
  private MotorDemo m_neomotor1;
  private MotorDemo m_neomotor2;
  private MotorDemo m_vortexmotor1;
  private LoggedNetworkString loggedMotorSelection =
      new LoggedNetworkString("Subsystems/ControllerMotorSelection");
  private Command selectACommand;
  private Command selectBCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        m_neomotor1 =
            new MotorDemo(new MotorDemoIOSparkFlex(MotorDemoConstants.devIDNEO1, 0.02), "NEO1");
        m_neomotor2 =
            new MotorDemo(new MotorDemoIOSparkFlex(MotorDemoConstants.devIDNEO2, 0.50), "NEO2");
        m_vortexmotor1 =
            new MotorDemo(
                new MotorDemoIOSparkMax(MotorDemoConstants.devIDVORTEX1, 0.50), "VORTEX1");
        break;

      case SIM:
        m_neomotor1 = new MotorDemo(new MotorDemoIOSim(MotorDemoConstants.devIDNEO1, 0.02), "NEO1");
        m_neomotor2 = new MotorDemo(new MotorDemoIOSim(MotorDemoConstants.devIDNEO2, 0.50), "NEO2");
        m_vortexmotor1 =
            new MotorDemo(new MotorDemoIOSim(MotorDemoConstants.devIDVORTEX1, 0.50), "VORTEX1");
        break;

      case REPLAY:
        break;

      default:
        break;
    }

    // Define the selector logic
    selectACommand =
        new SelectCommand(
            Map.ofEntries(
                Map.entry(Constants.MotorSelection.NEO1, m_neomotor1.goFast()),
                Map.entry(Constants.MotorSelection.NEO2, m_neomotor2.goFast()),
                Map.entry(Constants.MotorSelection.VORTEX1, m_vortexmotor1.goFast())),
            // This supplier determines which map entry to run
            () -> motorSel);

    selectBCommand =
        new SelectCommand(
            Map.ofEntries(
                Map.entry(Constants.MotorSelection.NEO1, m_neomotor1.goMid()),
                Map.entry(Constants.MotorSelection.NEO2, m_neomotor2.goMid()),
                Map.entry(Constants.MotorSelection.VORTEX1, m_vortexmotor1.goMid())),
            // This supplier determines which map entry to run
            () -> motorSel);

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
    controller.y().onTrue(Commands.runOnce(() -> motorSel = motorSel.next()));

    controller.a().whileTrue(selectACommand);
    controller.b().whileTrue(selectBCommand);

    m_neomotor1.setDefaultCommand(
        new ConditionalCommand(
            m_neomotor1.goFromSupplier(() -> -controller.getLeftY()), // Runs if condition is true
            m_neomotor1.goStop(), // Runs if condition is false
            () -> motorSel == Constants.MotorSelection.NEO1 // The condition
            ));

    m_neomotor2.setDefaultCommand(
        new ConditionalCommand(
            m_neomotor2.goFromSupplier(() -> -controller.getLeftY()), // Runs if condition is true
            m_neomotor2.goStop(), // Runs if condition is false
            () -> motorSel == Constants.MotorSelection.NEO2 // The condition
            ));

    m_vortexmotor1.setDefaultCommand(
        new ConditionalCommand(
            m_vortexmotor1.goFromSupplier(
                () -> -controller.getLeftY()), // Runs if condition is true
            m_vortexmotor1.goStop(), // Runs if condition is false
            () -> motorSel == Constants.MotorSelection.VORTEX1 // The condition
            ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public void updateNTValues() {
    loggedMotorSelection.set(motorSel.name());
  }
}
