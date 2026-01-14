// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controller;
import frc.robot.Constants.ID;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.StopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.sim.SimDrivetrain;
import frc.sim.SimLimelight;
import frc.sim.SimTarget;

import static edu.wpi.first.units.Units.Radian;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.Vector;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.*;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition Tab");

  private final Pigeon2 m_gyro = new Pigeon2(0);
  private final Drivetrain m_swerve;
  private final Limelight m_Limelight;

  Command m_driveCommand;

  private final SendableChooser<String> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    this.m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(0));
    if (Robot.isSimulation()) {
      this.m_swerve = new SimDrivetrain(m_gyro);
    } else {
      this.m_swerve = new Drivetrain(m_gyro);
    }

    if (Robot.isReal()) {
      this.m_Limelight = new Limelight();
    } else {
      Vector<SimTarget> targets = new Vector<SimTarget>();
      SimTarget target = new SimTarget((float) Units.Meters.convertFrom(144, Units.Inches),
          (float) Units.Meters.convertFrom(158.5, Units.Inches), 0.0f);
      targets.add(target);
      this.m_Limelight = new SimLimelight((SimDrivetrain) m_swerve, targets, true);
    }
    this.m_Limelight.setLimelightPipeline(LimelightConstants.defaultPipeline);

    // Xbox controllers return negative values when we push forward.
    this.m_driveCommand = new Drive(m_swerve);
    this.m_swerve.setDefaultCommand(this.m_driveCommand);

    autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()'

    configurePathPlanner();
    autoChooser.setDefaultOption("DO NOTHING!", "NO AUTO");
    m_competitionTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(7, 0);


    // Configure the trigger bindings
    configureBindings();

    if (Robot.isSimulation()) {
      ((SimDrivetrain) m_swerve).setSimPose(new Pose3d(new Translation3d(),
          new Rotation3d(0, 0, m_swerve.getExpectedStartGyro()*(3.14159/180.0))));
      System.out.println("Setting sim pose " + ((SimDrivetrain) m_swerve).getSimPose());
    }
  }

  public void setTeleDefaultCommand() {
    if (this.m_swerve.getDefaultCommand() == null) {
      this.m_swerve.setDefaultCommand(this.m_driveCommand);
    }
  }

  public void setAutoDefaultCommand() {
    if (this.m_swerve.getDefaultCommand() == null) {
      this.m_swerve.setDefaultCommand(this.m_driveCommand);
    }
  }

  public void clearDefaultCommand() {
    this.m_swerve.removeDefaultCommand();
  }

  public Drivetrain getDrive() {
    return this.m_swerve;
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  public void configureBindings() {

  }

  private void configurePathPlanner() {
    autoChooser.addOption("DriveForward", "DriveForward"); // Permanent choice
}

public void startAutonomous() {
  String auto = autoChooser.getSelected();
  SequentialCommandGroup start;
  if (auto.equals("DriveForward")) { // For testing
    start = new SequentialCommandGroup(
        getAutonomousCommand("DriveForward", true));
    start.schedule();
  } else {
    System.err.println("Invalid auto routine specified");
  }
}

  public Command getAutonomousCommand(String pathName, boolean resetOdometry) {
    List<Waypoint> waypoints;
    PathPlannerPath path;
    Waypoint first;
    System.out.println("getAutoCommand building auto for " + pathName);
    try {
      path = PathPlannerPath.fromPathFile(pathName);
      if (resetOdometry) {
        Optional<Pose2d> pose = path.getStartingHolonomicPose();
        Optional<Alliance> ally = DriverStation.getAlliance();
        waypoints = path.getWaypoints();
        first = waypoints.get(0);
        if (ally.isPresent()) {
          if (ally.get() == Alliance.Red) {
            System.out.println("Flipping start location for red");
            first = first.flip();
          }
        }
        if (pose.isPresent()) {
          m_swerve.resetStartingTranslation(first.anchor());
          System.out.println(first.toString());
        } else {
          System.out.println("Error getting PathPlanner pose");
        }
        if (Robot.isSimulation()) {
          ((SimDrivetrain) m_swerve).setSimPose(new Pose3d(new Translation3d(first.anchor()),
              new Rotation3d(0, 0, (m_swerve.getExpectedStartGyro()/180.0)*3.14159)));
          System.out.println("Setting sim pose " + ((SimDrivetrain) m_swerve).getSimPose());
        }
      }
      return AutoBuilder.followPath(path);
    } catch (FileVersionException | IOException | ParseException e) {
      System.err.println("Error loading PathPlanner path");
      e.printStackTrace();
    }
    return new StopDrive(m_swerve);
  }

  public void reportTelemetry() {

  }
}
