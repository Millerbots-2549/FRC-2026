// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public double targetAngle = 0.0;
    public double currentAngle = 0.0;
    public double currentSetpoint = 0.0;
    public double pivotVelocity = 0.0;
    public double appliedVolts = 0.0;
  }

  default void updateInputs(IntakePivotIOInputs inputs) {}

  default void setPivotOpenLoop(double output) {}

  default void setPivotAngle(Rotation2d target) {}

  default void resetAngle(double resetAngleDeg) {}

  default void periodic() {}
}
