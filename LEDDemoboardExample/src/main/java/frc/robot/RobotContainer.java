// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*; // imports everything in the commands folder


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final LightSubsystem m_lightSubsystem = new LightSubsystem();
    private final HubActivity cycleLED = new HubActivity(m_lightSubsystem);
    private final RainbowLED rainbowLED = new RainbowLED(m_lightSubsystem);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        m_lightSubsystem.setToColor(5, 200, 0, 0);
        m_lightSubsystem.setToColor(6, 0, 0, 200);
        m_lightSubsystem.setToColor(9, 0, 200, 0);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox} /
     * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {

        // Turn the lights on when button pressed. Turn off when button is released
       //m_driverController.b().onTrue(m_lightSubsystem.changeAllLEDColor(255, 0, 0));
        //m_driverController.b().onFalse(m_lightSubsystem.changeAllLEDColor(0, 0, 0));

        // Turn the lights on when button held. Turn off when button is released
        //m_driverController.y().whileTrue(m_lightSubsystem.changeAllLEDColor(255, 0, 255));
        //m_driverController.y().onFalse(m_lightSubsystem.changeAllLEDColor(0, 0, 0));
        
        // Cycle between 3 LED colors until interrupted
        m_driverController.a().toggleOnTrue(cycleLED);

        // Note: the setAllBlue command doesn't run when disabled.
        m_driverController.x().toggleOnTrue(rainbowLED);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        return null;
    }
    
    public boolean areLightsOn() {
        return m_lightSubsystem.areLightsOn();
    }
}
