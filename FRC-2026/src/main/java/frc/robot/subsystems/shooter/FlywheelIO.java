// Copyflywheel (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean flywheelConnected = false;
    public double flywheelVoltage;
    public double flywheelCurrent;
    public double flywheelTemperature;
    public double flywheelRPM;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setVelocity(double velocityRPM) {}
}
