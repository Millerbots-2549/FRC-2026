// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean feederConnected = false;
    public double feederVoltage;
    public double feederCurrent;
    public double feederRPM;
  }

  default void updateInputs(FeederIOInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setVelocity(double velocityRPM) {}
}
