// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.motors;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface MotorDemoIO {
  @AutoLog
  public static class MotorDemoIOInputs {
    public double appliedVolts = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MotorDemoIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double motorVolts) {}

  /** Run open loop at the specified voltage with a deadband adjustment */
  public default void setVoltageWithDeadband(DoubleSupplier motorVolts, double deadbandVolts) {}
}
