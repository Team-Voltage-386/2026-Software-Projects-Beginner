// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.motors;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

public class MotorDemoIOSparkMax implements MotorDemoIO {
  private SparkMax mainMotor;

  /* Constructor */
  public MotorDemoIOSparkMax(int deviceID) {
    mainMotor = new SparkMax(deviceID, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(MotorDemoIOInputs inputs) {
    inputs.appliedVolts = mainMotor.getAppliedOutput() * mainMotor.getBusVoltage();
  }

  @Override
  public void setVoltage(double motorVolts) {
    mainMotor.setVoltage(motorVolts);
  }

  @Override
  public void setVoltageWithDeadband(DoubleSupplier motorVolts, double deadbandVolts) {
    double motorDoubleValue = MathUtil.applyDeadband(motorVolts.getAsDouble(), deadbandVolts);

    motorDoubleValue = motorDoubleValue * mainMotor.getBusVoltage();
    mainMotor.setVoltage(motorDoubleValue);
  }
}
