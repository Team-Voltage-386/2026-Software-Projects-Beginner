package frc.robot.subsystems.motors;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class MotorDemo extends SubsystemBase {
  private final MotorDemoIO io;
  private final MotorDemoIOInputsAutoLogged inputs = new MotorDemoIOInputsAutoLogged();
  private String demoString = "NONE";

  // Define gains (example values, replace with your own)
  private static final double kS = 0.1; // Static friction voltage
  private static final double kG = 0.5; // Gravity gain (voltage needed to hold horizontally)
  private static final double kV = 2.0; // Velocity gain (volts per radian/sec)
  private static final double kA = 0.1; // Acceleration gain (volts per radian/sec^2)
  private final ArmFeedforward armfeedforward = new ArmFeedforward(kS, kG, kV, kA);

  // Define PID controller with example gains (these need tuning)
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private final PIDController pidcontroller = new PIDController(kP, kI, kD);

  private double voltsFF = 0.0;
  private double voltsPID = 0.0;

  public MotorDemo(MotorDemoIO io, String demoString) {
    this.io = io;
    setDemoString(demoString);
  }

  public Command goFast() {
    return run(() -> io.setVoltage(12.0));
  }

  public Command goMid() {
    return run(() -> io.setVoltage(6.0));
  }

  public Command goSlow() {
    return run(() -> io.setVoltage(3.0));
  }

  public Command goStop() {
    return run(() -> io.setVoltage(0.0));
  }

  public Command goFromSupplier(DoubleSupplier voltsSupplier) {
    return runOnce(() -> io.setVoltageWithDeadband(voltsSupplier, MotorDemoConstants.DEADBAND));
  }

  public double getArmPosition() {
    return io.getPosition();
  }

  public Command goUsingFFandPID(DoubleSupplier voltsSupplier) {
    voltsFF = armfeedforward.calculate(Math.toRadians(90), 10);
    voltsPID = pidcontroller.calculate(getArmPosition(), voltsSupplier.getAsDouble());
    return runOnce(() -> io.setVoltage(voltsFF + voltsPID));
  }

  public void setDemoString(String inputString) {
    demoString = inputString;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("MotorDemo/" + demoString, inputs);
  }
}
