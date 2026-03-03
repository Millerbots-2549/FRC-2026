// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean flywheelConnected = false;
    public double flywheelVoltage;
    public double flywheelCurrent;
    public double flywheelTemperature;
    public double flywheelRPM;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}
  ;

  /**
   * Runs the roller motor to run at a specific velocity in <b>rotations per second</b>
   *
   * @param velocity The velocity in <b>rotations per second</b>
   */
  public default void setRollerVelocity(double velocity) {}
  ;

  /**
   * Runs the roller motor to run at a specific voltage
   *
   * @param volts The voltage to run the motor at
   */
  public default void setRollerOpenLoop(double volts) {}
  ;
}
