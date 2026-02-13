// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.motors;

import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

public class MotorDemoIOSim implements MotorDemoIO {
  private double lastAppliedVolts;
  private int lastDeviceID;

  /* Constructor */
  public MotorDemoIOSim(int deviceID) {
    lastDeviceID = deviceID;
    lastAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(MotorDemoIOInputs inputs) {
    inputs.appliedVolts = lastAppliedVolts;
  }

  @Override
  public void setVoltage(double motorVolts) {
    lastAppliedVolts = motorVolts;
  }

  @Override
  public void setVoltageWithDeadband(DoubleSupplier motorVolts, double deadbandVolts) {
    double motorDoubleValue = MathUtil.applyDeadband(motorVolts.getAsDouble(), deadbandVolts);

    motorDoubleValue = motorDoubleValue * 12;
    lastAppliedVolts = motorDoubleValue;
  }

  public int getDeviceID() {
    return lastDeviceID;
  }
}
