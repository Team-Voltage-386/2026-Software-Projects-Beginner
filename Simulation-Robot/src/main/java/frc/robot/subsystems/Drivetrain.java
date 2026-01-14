// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {

    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // per second

    protected final Pigeon2 m_gyro;

    protected boolean fieldRelative = true;
    protected final ShuffleboardTab m_driveTab = Shuffleboard.getTab("drive subsystem");
    protected final SimpleWidget m_fieldRelativeWidget = m_driveTab.add("drive field relative", fieldRelative);
    protected final GenericEntry m_driveCommandedRotationSpeed = m_driveTab.add("drive commanded rotation", 0)
            .getEntry();


    protected final SwerveDrivePoseEstimator m_odometry;
    protected int counter = 0;
    protected boolean enableVisionPoseInputs;

    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    protected ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();

    protected ExecutorService executorService = Executors.newFixedThreadPool(4);

    protected final Field2d field = new Field2d();

    protected final Translation2d m_frontLeftLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXForward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYLeft);
    protected final Translation2d m_frontRightLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXForward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYRight);
    protected final Translation2d m_backLeftLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXBackward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYLeft);
    protected final Translation2d m_backRightLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXBackward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYRight);

    /**
     * The order that you initialize these is important! Later uses of functions
     * like toSwerveModuleStates will return the same order that these are provided.
     * See
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    protected final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);


    public Drivetrain(Pigeon2 gyro) {
        // Zero at beginning of match. Zero = whatever direction the robot (more
        // specifically the gyro) is facing
        this.m_gyro = gyro;
        m_driveTab.add("field", field);

        m_odometry = new SwerveDrivePoseEstimator(
                m_kinematics,
                getGyroYawRotation2d(),
                new SwerveModulePosition[] { new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition() },
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

        // Load the RobotConfig from the PathPlanner GUI settings
        RobotConfig ppConfig;
        try {
            ppConfig = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getRoboPose2d, // Robot pose supplier
                    this::resetOdo, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    this::driveChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    new PPHolonomicDriveController( // HolonomicPathFollowerConfig
                            new PIDConstants(DrivetrainConstants.drivePID[0], // Translation PID constants
                                    DrivetrainConstants.drivePID[1],
                                    DrivetrainConstants.drivePID[2]),
                            new PIDConstants(DrivetrainConstants.turnPID[0], // Rotation PID constants
                                    DrivetrainConstants.turnPID[1],
                                    DrivetrainConstants.turnPID[2])),
                    ppConfig,
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE            
                        return getAlliance() == DriverStation.Alliance.Red;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }
    
    public void resetStartingPose(Pose2d newPose) {
        m_odometry.resetPose(new Pose2d());
    }

    public void resetStartingTranslation(Translation2d newTranslation) {
        m_odometry.resetTranslation(newTranslation);
    }

    public Pigeon2 getGyro() {
        return m_gyro;
    }

    public Alliance getAlliance() {
        Optional<Alliance> myAlliance = DriverStation.getAlliance();
        if (myAlliance.isPresent()) {
            return myAlliance.get();
        } else {
            System.out.println("No alliance present. Returning red.");
            return Alliance.Red;
        }
    }

    /**
     * Resets Orientation of the robot
     */
    public void resetGyro() {
        if (getAlliance() == Alliance.Blue) {
            System.out.println("Initializing gyro to 180 for BLUE");
            m_gyro.setYaw(180.0);
        } else {
            System.out.println("Initializing gyro to 0 for RED");
            m_gyro.setYaw(0);
        }
    }

    public double getExpectedStartGyro() {
        if (getAlliance() == DriverStation.Alliance.Blue) {
            return 180.0;
        } else {
            return 0.0;
        }
    }

    /**
     * Resets robot position on the field
     */
    public void resetOdo() {
        System.out.println("Calling resetOdo with no arguments");
    }

    public void updateOdometry() {

    }

    /**
     * Resets Odometry using a specific Pose2d
     * 
     * @param pose
     */
    public void resetOdo(Pose2d pose) {
        if (pose != null) {
            System.out.println("Calling resetOdo with pose " + pose);
            m_odometry.resetPose(new Pose2d());
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }

    public void setFieldRelative(boolean isFieldRelative) {
        fieldRelative = isFieldRelative;
        m_fieldRelativeWidget.getEntry().setBoolean(fieldRelative);
    }

    public Command toggleFieldRelativeCommand() {
        return runOnce(() -> {
            this.setFieldRelative(!this.fieldRelative);
        });
    }

    /**
     * Get the yaw of gyro in Rotation2d form
     * 
     * @return chasis angle in Rotation2d
     */
    public Rotation2d getGyroYawRotation2d() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
    }

    public Rotation2d getPlayerStationRelativeYaw2d() {
        Rotation2d rot = Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
        if (getAlliance() == DriverStation.Alliance.Red) {
            rot = rot.rotateBy(Rotation2d.k180deg);
        }
        return rot;
    }

    public ChassisSpeeds getCommandeChassisSpeeds() {
        return commandedChassisSpeeds;
    }

    protected double driveMultiplier = 1;

    public void setDriveMult(double mult) {
        driveMultiplier = mult;
    }

    public Command setDriveMultCommand(double mult) {
        return Commands.runOnce(() -> setDriveMult(mult));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotSpeed      Angular rate of the robot.
     * 
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        xSpeed = xSpeed * driveMultiplier;
        ySpeed = ySpeed * driveMultiplier;
        rotSpeed = rotSpeed * driveMultiplier;

        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                    getPlayerStationRelativeYaw2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        driveChassisSpeeds(chassisSpeeds);

        SmartDashboard.putNumber("desired X speed", xSpeed);
        SmartDashboard.putNumber("desired Y speed", ySpeed);
        SmartDashboard.putNumber("dsired rot speed", rotSpeed);
    }

    public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        m_driveCommandedRotationSpeed.setDouble(Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));
        commandedChassisSpeeds = chassisSpeeds;
    }

    public Pose2d getRoboPose2d() {
        return m_odometry.getEstimatedPosition();
    }

    public void setEnableVisionPoseInputs(boolean enableVisionPoseInputs) {
        this.enableVisionPoseInputs = enableVisionPoseInputs;
    }

    public void stopDriving() {
        commandedChassisSpeeds = new ChassisSpeeds();
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
