// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class ShooterConstants {
  public static final int HOOD_SERVO_CHANNEL = 0;

  /** in [mm] */
  public static final double HOOD_COUPLE_DISTANCE = 171.45;
  /** in [mm] */
  public static final double SERVO_COUPLE_DISTANCE = 185.0;
  /** in [radians] */
  public static final double SERVO_COUPLE_ANGLE = 0.0;
  /** in [mm] */
  public static final int SERVO_MAX_LENGTH = 140;
  /** in [mm] */
  public static final int SERVO_DEFAULT_LENGTH = 108;

  public static final double FLYWHEEL_STUCK_CURRENT_VEL_RATIO = 40 / 1;
  public static final double FLYWHEEL_VELOCITY_THRESHOLD = 15.0;

  public static final double FLYWHEEL_AGITATE_VELOCITY = 150.0;
}
