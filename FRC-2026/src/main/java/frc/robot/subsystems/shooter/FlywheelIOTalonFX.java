// Copyflywheel (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_VELOCITY_THRESHOLD;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final VoltageOut leftMotorVoltageControl = new VoltageOut(0);
  private final VoltageOut rightMotorVoltageControl = new VoltageOut(0);

  private final VelocityVoltage leftMotorVelocityControl = new VelocityVoltage(0);
  private final VelocityVoltage rightMotorVelocityControl = new VelocityVoltage(0);

  public static final double SHOOTER_GEAR_RATIO = 2.0 / 1.0;

  public FlywheelIOTalonFX() {
    leftMotor = new TalonFX(22);
    rightMotor = new TalonFX(57);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

    config.Slot0.kV = 0.22;
    config.Slot0.kP = 3.4;

    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.flywheelConnected =
        BaseStatusSignal.refreshAll(
                leftMotor.getMotorVoltage(),
                leftMotor.getSupplyCurrent(),
                leftMotor.getDeviceTemp(),
                leftMotor.getVelocity())
            .isOK();
    inputs.flywheelVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.flywheelCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.flywheelTemperature = leftMotor.getDeviceTemp().getValueAsDouble();
    inputs.flywheelRPM =
        (leftMotor.getRotorVelocity().getValueAsDouble() * 60) / SHOOTER_GEAR_RATIO;
  }

  @Override
  public void setVoltage(double output) {
    leftMotor.setControl(leftMotorVoltageControl.withOutput(output));
    rightMotor.setControl(rightMotorVoltageControl.withOutput(-output));
  }

  private double appliedRPM = 0.0;

  @Override
  public void setVelocity(double velocityRPM) {
    appliedRPM = velocityRPM * SHOOTER_GEAR_RATIO / 60.0;

    leftMotor.setControl(leftMotorVelocityControl.withVelocity(appliedRPM));
    rightMotor.setControl(rightMotorVelocityControl.withVelocity(-appliedRPM));
  }

  @Override
  public boolean atSpeed() {
    return Math.abs(leftMotor.getRotorVelocity().getValueAsDouble() - appliedRPM)
        < FLYWHEEL_VELOCITY_THRESHOLD;
  }
}
