// Copyflywheel (c) FIRST and other WPILib contributors.
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
public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX motor;

  private final VoltageOut motorVoltageControl = new VoltageOut(0);

  private final VelocityVoltage motorVelocityControl = new VelocityVoltage(0);

  public static final double SHOOTER_GEAR_RATIO = 1.0 / 5.0;

  public FlywheelIOTalonFX() {
    motor = new TalonFX(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

    config.Slot0.kV = 0.14;
    config.Slot0.kP = 0.4;
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
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
    inputs.flywheelRPM = (motor.getRotorVelocity().getValueAsDouble() * 60) / SHOOTER_GEAR_RATIO;
  }

  @Override
  public void setVoltage(double output) {
    motor.setControl(motorVoltageControl.withOutput(output));
  }

  @Override
  public void setVelocity(double velocityRPM) {
    double applied = velocityRPM * SHOOTER_GEAR_RATIO / 60.0;

    motor.setControl(motorVelocityControl.withVelocity(applied));
  }
}
