// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class LimelightConstants {
    public static final int defaultPipeline = 2;
    // For CenterOnTag
    public static final double minXVelocity = 0.1;
    public static final double maxXVelocity = 1.0;
    public static final double minYVelocity = 0.1;
    public static final double maxYVelocity = 1.0;
    public static final double minAngVelocityDPS = 0;
    public static final double maxAngVelocityDPS = 20;
    public static final double xDisThreshold = 0.03;
    public static final double rotThreshold = 1.0;
    public static final double azimuthFieldOfViewDeg = 29.0;
    public static final double xOffset = 0.0;
    public static final double yOffset = 0.7;
    public static final double maxAngAccMSS = 8;
    public static final double maxAngDccMSS = 16;
    // For ProportionalController & DriveDistance as well
    public static final double maxAccMSS = 3;
    public static final double maxDccMSS = 8;
    public static final double minVelocity = 0.1;
    public static final double maxVelocity = 5.0;
    public static final double offset = 0.0;
    public static final double proportion = 2;
    public static final double threshold = .02;
    public static final double driveDistanceProp = 3;
    // For DriveOffset
    public static final double driveOffsetXOffset = 0.7;
    public static final double driveOffsetYOffset = 0.17;
    public static final double driveOffsetMaxAccMSS = 2.5;
    public static final double driveOffsetMaxDccMSS = 8;
    public static final double driveOffsetMinVel = 0.1;
    public static final double driveOffsetMaxVel = 3.0;
    public static final double driveOffsetAngleError = 0.04;
    public static final double driveOffsetRangeMThreshold = 0.01;
    public static final double driveOffsetKp = 3.0;
}

public static class Offsets {
  // Camera Positioning
  public static final double cameraOffsetForwardM = 0.08;
  public static final double cameraOffsetFromFrontBumber = 0.38;
}

public static class DrivetrainConstants {
  // Distance in meters
  public static final double kDistanceMiddleToFrontMotor = 0.339852;
  public static final double kDistanceMiddleToSideMotor = 0.289052;
  public static final double kDriveBaseRadius = Math.sqrt( // distance from the middle to the furthest wheel
          kDistanceMiddleToFrontMotor * kDistanceMiddleToFrontMotor +
                  kDistanceMiddleToSideMotor * kDistanceMiddleToSideMotor);

  public static final int kXForward = 1;
  public static final int kXBackward = -1;
  public static final int kYLeft = 1;
  public static final int kYRight = -1;

  // ITS TUNED. NO TOUCH!
  public static final double[] turnPID = { 4.5, 1.0, 0.0 };
  public static final double[] drivePID = { 2, 0.0, 0.0 };
  public static final double[] turnFeedForward = { 0.3, 0.2 };
  public static final double[] driveFeedForward = { 0.17, 2.255 };

  public static final boolean kInvertTurn = true;
  public static final double kMaxPossibleSpeed = 5.3; // meters per second

};

public static class Controller {
    public static final int kDriveControllerID = 0;
    public static final int kManipControllerID = 1;

    /**
     * Rate limiters make joystick inputs more gentle; 1/3 sec from 0 to 1.
     */
    public static final double kRateLimitXSpeed = 150.0;
    public static final double kRateLimitYSpeed = 150.0;
    public static final double kRateLimitRot = 70.0;
    public static final double kMaxNecessarySpeed = DrivetrainConstants.kMaxPossibleSpeed * 0.8;

    public static final CommandXboxController kDriveController = new CommandXboxController(kDriveControllerID);
    public static final CommandXboxController kManipulatorController = new CommandXboxController(
            kManipControllerID);
}

public static class Deadbands {
  public static final double kLeftJoystickDeadband = 0.06;
  public static final double kRightJoyStickDeadband = 0.06;
}

public static class ID {

  // Limelight
  public static final String kFrontLimelightName = "limelight-c";

}

}
