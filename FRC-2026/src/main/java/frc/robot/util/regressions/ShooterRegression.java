// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.regressions;

import frc.lib.data.InterpolatingDouble;
import frc.lib.data.InterpolatingTreeMap;
import frc.lib.math.PolynomialRegression;

/** Add your docs here. */
public class ShooterRegression {
  public static final double HOOD_PADDING_DEGREES = 2;
  public static final double SHOOTER_PADDING_VELOCITY = 100;

  public static final double[] PADDING = {SHOOTER_PADDING_VELOCITY, HOOD_PADDING_DEGREES};

  // hood
  public static double defaultHoodAngle = Math.toRadians(0);
  public static boolean useHoodAimPolynomial = false;

  public static boolean useSmartDashboard = false;

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodAutoAimMap =
      new InterpolatingTreeMap<>();
  public static PolynomialRegression hoodAutoAimPolynomial;

  public static double[][] hoodRegression;

  static {
    hoodRegression = RegressionConstants.HOOD_MANUAL_ANGLE;

    for (double[] pair : hoodRegression) {
      hoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }

    hoodAutoAimPolynomial = new PolynomialRegression(hoodRegression, 1);
  }

  // shooter
  public static double defaultShootingRPM = 2950.0;
  public static boolean useFlywheelAutoAimPolynomial = false;

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelAutoAimMap =
      new InterpolatingTreeMap<>();
  public static PolynomialRegression flywheelAutoAimPolynomial;

  public static double[][] flywheelRegression;

  static {
    flywheelRegression = RegressionConstants.FLYWHEEL_MANUAL_RPM;

    for (double[] pair : flywheelRegression) {
      flywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }

    flywheelAutoAimPolynomial = new PolynomialRegression(flywheelRegression, 2);
  }
}
