// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.data.InterpolatingDouble;
import frc.lib.data.InterpolatingTreeMap;
import frc.lib.math.PolynomialRegression;
import frc.robot.util.regressions.ShooterRegression;

public class Shooter extends SubsystemBase {
  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final FeederIO feederIO;
  private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  private final PolynomialRegression flywheelSpeedRegression =
      ShooterRegression.flywheelAutoAimPolynomial;
  private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      flywheelSpeedInterpolator = ShooterRegression.flywheelAutoAimMap;

  /** Creates a new ShooterSubsystem. */
  public Shooter(FlywheelIO flywheelIO, FeederIO feederIO) {
    this.flywheelIO = flywheelIO;
    this.feederIO = feederIO;
  }

  public void applyAutoAimDistance(double distanceMeters) {
    double flywheelSpeed;
    if (ShooterRegression.useFlywheelAutoAimPolynomial) {
      flywheelSpeed = flywheelSpeedRegression.predict(distanceMeters);
    } else {
      flywheelSpeed =
          flywheelSpeedInterpolator.getInterpolated(new InterpolatingDouble(distanceMeters)).value;
    }

    flywheelIO.setVelocity(flywheelSpeed);
  }

  public void setFeederVelocity(double RPM) {
    feederIO.setVelocity(RPM);
  }

  public void setVelocity(double vel) {
    flywheelIO.setVelocity(vel);
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    feederIO.updateInputs(feederInputs);
  }

  public double getFlywheelVelocity() {
    return flywheelInputs.flywheelRPM;
  }

  public double getFlywheelCurrent() {
    return flywheelInputs.flywheelCurrent;
  }

  public boolean shooterAtSpeed() {
    return flywheelIO.atSpeed();
  }
}
