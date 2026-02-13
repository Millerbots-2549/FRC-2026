// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean hoodConnected = false;
    public Rotation2d hoodAngle = Rotation2d.kZero;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Sets the desired angle of the shooter */
  public default void applySetpoint(Rotation2d setpoint) {}

  /** */
  public default Rotation2d getAngle() {
    return Rotation2d.kZero;
  }
}
