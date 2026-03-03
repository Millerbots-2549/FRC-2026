// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public final class IntakeConstants {
  public static final int PIVOT_MOTOR_ID = 62;
  public static final int ROLLER_MOTOR_ID = 16;

  public static final double PIVOT_MAX_DEGREES = 99.58;
  public static final double PIVOT_MIN_DEGREES = 0;

  public static final double PIVOT_GEAR_RATIO = 360.0 / 5.0;

  public static final double PIVOT_KP = 0.34;
  public static final double PIVOT_KI = 0.0;
  public static final double PIVOT_KD = 0.0;

  public static final double PIVOT_KS = 0.0;
  public static final double PIVOT_KV = 0.0;
  public static final double PIVOT_KA = 0.0;
  public static final double PIVOT_KCOS = 0.1;

  public static final double PIVOT_MAX_VELOCITY = 2.0;
  public static final double PIVOT_MAX_ACCELERATION = 2.0;

  public static final double ROLLER_MAX_SPEED = 0.62;
}
