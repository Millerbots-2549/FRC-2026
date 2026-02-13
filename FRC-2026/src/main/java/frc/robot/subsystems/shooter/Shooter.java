// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.data.InterpolatingDouble;
import frc.lib.data.InterpolatingTreeMap;
import frc.lib.math.PolynomialRegression;
import frc.robot.util.regressions.ShooterRegression;

public class Shooter extends SubsystemBase {
  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final PolynomialRegression hoodAngleRegression = ShooterRegression.hoodAutoAimPolynomial;
  private final PolynomialRegression flywheelSpeedRegression = ShooterRegression.flywheelAutoAimPolynomial;
  private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodAngleInterpolator = ShooterRegression.hoodAutoAimMap;
  private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelSpeedInterpolator = ShooterRegression.flywheelAutoAimMap;

  /** Creates a new ShooterSubsystem. */
  public Shooter(HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;
  }

  public void applyHoodSetpoint(Rotation2d setpoint) {
    hoodIO.applySetpoint(setpoint);
  }

  private double setpoint = 0.0;

  public void sustainHood() {
    hoodIO.applySetpoint(Rotation2d.fromRadians(setpoint));
  }

  public void raiseHood() {
    setpoint += 0.02;
    sustainHood();
  }

  public void lowerHood() {
    setpoint -= 0.02;
    sustainHood();
  }

  public void applyAutoAimDistance(double distanceMeters) {
    double hoodAngle;
    if(ShooterRegression.useHoodAimPolynomial) {
      hoodAngle = hoodAngleRegression.predict(distanceMeters);
    } else {
      hoodAngle = hoodAngleInterpolator.getInterpolated(new InterpolatingDouble(distanceMeters)).value;
    }
    double flywheelSpeed;
    if(ShooterRegression.useHoodAimPolynomial) {
      flywheelSpeed = flywheelSpeedRegression.predict(distanceMeters);
    } else {
      flywheelSpeed = flywheelSpeedInterpolator.getInterpolated(new InterpolatingDouble(distanceMeters)).value;
    }

    hoodIO.applySetpoint(Rotation2d.fromRadians(hoodAngle));
    flywheelIO.setVelocity(flywheelSpeed);
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    flywheelIO.updateInputs(flywheelInputs);
  }
}
