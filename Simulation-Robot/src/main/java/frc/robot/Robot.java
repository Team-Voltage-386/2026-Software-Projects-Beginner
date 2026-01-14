// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.sim.SimDrivetrain;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_container;
  protected boolean gyroCorrect;
  protected Alliance currentAlliance;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    gyroCorrect = false;
    m_container = new RobotContainer();
    currentAlliance = m_container.getDrive().getAlliance();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_container.reportTelemetry();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (currentAlliance != m_container.getDrive().getAlliance()) {
      currentAlliance = m_container.getDrive().getAlliance();
      System.out.println("Alliance changed to " + currentAlliance);
      gyroCorrect = false;
  }
  if (!gyroCorrect) {
      double expectedGyro = m_container.getDrive().getExpectedStartGyro();
      double currentGyro = m_container.getDrive().getGyroYawRotation2d().getDegrees();
      System.out.println("Expected Gyro: " + expectedGyro + " current: " + currentGyro);
      if (Math.abs(expectedGyro - currentGyro) > 1.0) {
          System.out.println("Gyro delta too large");
          m_container.getDrive().resetGyro();
          System.out.println("Reset gyro from robot periodic");
      } else {
          gyroCorrect = true;
          m_container.getDrive().resetOdo();
      }
  }

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if(Robot.isSimulation()) {
      ((SimDrivetrain)m_container.getDrive()).reset();
    }
    m_container.clearDefaultCommand();
    m_container.setAutoDefaultCommand();
    m_container.startAutonomous();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_container.clearDefaultCommand();
    m_container.setTeleDefaultCommand();
    m_container.configureBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
