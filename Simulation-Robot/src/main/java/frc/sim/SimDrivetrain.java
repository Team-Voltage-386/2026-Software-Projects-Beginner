package frc.sim;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.subsystems.Drivetrain;

public class SimDrivetrain extends Drivetrain {

    protected float desiredXSpeedMps = 0;
    protected float desiredYSpeedMps = 0;
    protected float desiredRotationSpeedDps = 0;
    protected float currentXSpeedMps = 0;
    protected float currentYSpeedMps = 0;
    protected float currentRotSpeedDps = 0;

    protected Pose3d currentPose = new Pose3d();
    protected Pigeon2SimState gyroSimState;

    protected final double loopTime = 0.02;

    protected final ShuffleboardTab m_driveTab = Shuffleboard.getTab("drive subsystem");
    protected final GenericEntry m_gyroSimYaw = m_driveTab.add("GyroSimYaw", 0).getEntry();
    
    public SimDrivetrain(Pigeon2 gyro)
    {
        super(gyro);
        gyroSimState = gyro.getSimState();
    }

    public void reset() {
        this.commandedChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d());
        this.currentXSpeedMps = 0.0f;
        this.currentYSpeedMps = 0.0f;
        this.currentRotSpeedDps = 0.0f;
    }

    @Override
    public void simulationPeriodic() {
        // Convert chassis reference to field relative
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(commandedChassisSpeeds, this.currentPose.getRotation().toRotation2d());

        // Update position and velocity
        this.currentXSpeedMps = (float)fieldSpeeds.vxMetersPerSecond;
        this.currentYSpeedMps = (float)fieldSpeeds.vyMetersPerSecond;
        this.currentRotSpeedDps = (float)Math.toDegrees(commandedChassisSpeeds.omegaRadiansPerSecond);

        this.currentPose = new Pose3d(currentPose.getX() + currentXSpeedMps*loopTime,
            currentPose.getY() + currentYSpeedMps*loopTime, 0.0,
            new Rotation3d(0.0, 0.0, currentPose.getRotation().getZ() + Math.toRadians(currentRotSpeedDps*loopTime)));  
        gyroSimState.addYaw(currentRotSpeedDps*loopTime);
        m_gyroSimYaw.setDouble(this.getGyroYawRotation2d().getDegrees());
    }

    public Pose3d getSimPose() {
        return this.currentPose;
    }

    public void setSimPose(Pose3d newPose) {
        this.currentPose = newPose;
    }

    @Override
    public void updateOdometry() {
        // For the odometry to the simulated robot position since we are
        // not simulating the actual swerve drive dynamics yet
        m_odometry.resetPose(getSimPose().toPose2d());
        m_odometry.resetRotation(getSimPose().getRotation().toRotation2d());

        // Set the chassis speeds to the true simulated speeds
        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(this.currentXSpeedMps,
            this.currentYSpeedMps, Units.degreesToRadians(this.currentRotSpeedDps),  getSimPose().getRotation().toRotation2d());

        field.setRobotPose(getRoboPose2d());
    }
}
