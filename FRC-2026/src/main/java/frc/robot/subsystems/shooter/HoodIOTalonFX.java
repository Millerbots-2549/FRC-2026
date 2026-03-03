// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LinearServo;

/** Add your docs here. */
public class HoodIOTalonFX implements HoodIO {
  private final double b = HOOD_COUPLE_DISTANCE;
  private final double c = SERVO_COUPLE_DISTANCE;
  private final double b2 = b * b;
  private final double c2 = c * c;

  private LinearServo hoodLinearActuatorLeft;
  private LinearServo hoodLinearActuatorRight;

  public HoodIOTalonFX() {
    hoodLinearActuatorLeft = new LinearServo(0, SERVO_MAX_LENGTH, 2000);
    hoodLinearActuatorRight = new LinearServo(1, SERVO_MAX_LENGTH, 2000);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodLinearActuatorLeft.updateCurPos();
    hoodLinearActuatorRight.updateCurPos();
    inputs.hoodAngle = getAngle();
  }

  @Override
  public void applySetpoint(Rotation2d setpoint) {
    double l =
        Math.sqrt(b2 + c2 - (2 * b * c * Math.cos(setpoint.getRadians() - SERVO_COUPLE_ANGLE)));
    //        - SERVO_DEFAULT_LENGTH;
    l = MathUtil.clamp(l, 0, SERVO_MAX_LENGTH);
    hoodLinearActuatorLeft.setPosition(l);
    hoodLinearActuatorRight.setPosition(l);

    SmartDashboard.putNumber("angle", setpoint.getDegrees());
    SmartDashboard.putNumber("length", l);
  }

  @Override
  public Rotation2d getAngle() {
    hoodLinearActuatorLeft.updateCurPos();
    hoodLinearActuatorRight.updateCurPos();
    double a = hoodLinearActuatorLeft.getPosition() + SERVO_DEFAULT_LENGTH;
    double angleRad = 2 * Math.acos(((a * a) - b2 - c2) / (-2.0 * b * c)) - SERVO_COUPLE_ANGLE;
    return Rotation2d.fromRadians(angleRad);
  }
}
