// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class FeederIOFalcon implements FeederIO {
  private final TalonFX motor;

  private final VoltageOut motorVoltageControl = new VoltageOut(0);

  private final VelocityVoltage motorVelocityControl = new VelocityVoltage(0);

  public FeederIOFalcon() {
    motor = new TalonFX(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

    config.Slot0.kV = 0.22;
    config.Slot0.kP = 2.8;

    motor.getConfigurator().apply(config);
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feederConnected =
        BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(),
                motor.getSupplyCurrent(),
                motor.getDeviceTemp(),
                motor.getVelocity())
            .isOK();
    inputs.feederVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.feederCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.feederRPM = motor.getRotorVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void setVoltage(double output) {
    motor.setControl(motorVoltageControl.withOutput(output));
  }

  private double appliedRPM = 0.0;

  @Override
  public void setVelocity(double velocityRPM) {
    appliedRPM = velocityRPM / 60.0;

    motor.setControl(motorVelocityControl.withVelocity(appliedRPM));
  }
}
