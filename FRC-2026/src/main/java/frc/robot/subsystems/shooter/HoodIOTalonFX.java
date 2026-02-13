// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LinearServo;

/** Add your docs here. */
public class HoodIOTalonFX implements HoodIO {
  private final double b = HOOD_COUPLE_DISTANCE;
  private final double c = SERVO_COUPLE_DISTANCE;

  private LinearServo hoodLinearActuator;

  public HoodIOTalonFX() {
    hoodLinearActuator = new LinearServo(0, SERVO_MAX_LENGTH, 2000);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodLinearActuator.updateCurPos();
    inputs.hoodAngle = getAngle();
  }

  @Override
  public void applySetpoint(Rotation2d setpoint) {
    hoodLinearActuator.setPosition(
        Math.sqrt(
                -2 * b * c * Math.cos(setpoint.getRadians() + SERVO_COUPLE_ANGLE)
                    + (b * b)
                    + (c * c))
            - SERVO_DEFAULT_LENGTH);
  }

  @Override
  public Rotation2d getAngle() {
    hoodLinearActuator.updateCurPos();
    double a = hoodLinearActuator.getPosition() + SERVO_DEFAULT_LENGTH;
    double angleRad =
        2 * Math.acos(((a * a) - (b * b) - (c * c)) / (-2.0 * b * c)) - SERVO_COUPLE_ANGLE;
    return Rotation2d.fromRadians(angleRad);
  }
}
