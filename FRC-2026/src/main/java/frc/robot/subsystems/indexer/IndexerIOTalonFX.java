// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class IndexerIOTalonFX implements IndexerIO {
  protected final TalonFX motor;

  private final VoltageOut motorVoltageControl = new VoltageOut(0);

  private final VelocityVoltage motorVelocityControl = new VelocityVoltage(0);

  public IndexerIOTalonFX() {
    motor = new TalonFX(61);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

    config.Slot0.kV = 0.14;
    config.Slot0.kP = 3.4;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.flywheelConnected =
        BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(),
                motor.getSupplyCurrent(),
                motor.getDeviceTemp(),
                motor.getVelocity())
            .isOK();
    inputs.flywheelVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.flywheelCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.flywheelTemperature = motor.getDeviceTemp().getValueAsDouble();
    inputs.flywheelRPM = (motor.getRotorVelocity().getValueAsDouble() * 60);
  }

  @Override
  public void setRollerVelocity(double velocity) {
    double applied = velocity / 60.0;

    motor.setControl(motorVelocityControl.withVelocity(applied));
  }

  @Override
  public void setRollerOpenLoop(double volts) {
    motor.setControl(motorVoltageControl.withOutput(volts));
  }
}
