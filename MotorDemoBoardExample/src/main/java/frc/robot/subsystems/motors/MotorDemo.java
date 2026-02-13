package frc.robot.subsystems.motors;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class MotorDemo extends SubsystemBase {
  private final MotorDemoIO io;
  private final MotorDemoIOInputsAutoLogged inputs = new MotorDemoIOInputsAutoLogged();
  private String demoString = "NONE";

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

  public void setDemoString(String inputString) {
    demoString = inputString;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("MotorDemo/" + demoString, inputs);
  }
}
