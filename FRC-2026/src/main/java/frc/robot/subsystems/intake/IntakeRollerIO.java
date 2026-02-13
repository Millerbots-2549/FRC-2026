// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public double rollerVelocity = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrent = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}
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
