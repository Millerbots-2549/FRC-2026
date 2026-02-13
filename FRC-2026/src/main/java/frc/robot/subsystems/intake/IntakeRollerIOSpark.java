// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.ROLLER_MOTOR_ID;
import static frc.robot.util.PhoenixUtil.ifOk;
import static frc.robot.util.PhoenixUtil.sparkStickyFault;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class IntakeRollerIOSpark implements IntakeRollerIO {
  protected final SparkMax rollerMotor;
  protected final RelativeEncoder rollerEncoder;

  public IntakeRollerIOSpark(SparkMaxConfig pivotConfig) {
    rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getEncoder();

    this.rollerMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tryUntilOk(rollerMotor, 5, () -> rollerEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(rollerMotor, rollerEncoder::getVelocity, (value) -> inputs.rollerVelocity = value);
    ifOk(
        rollerMotor,
        new DoubleSupplier[] {rollerMotor::getAppliedOutput, rollerMotor::getBusVoltage},
        (values) -> inputs.rollerAppliedVolts = values[0] * values[1]);
  }

  @Override
  public void setRollerVelocity(double velocity) {}

  @Override
  public void setRollerOpenLoop(double volts) {
    rollerMotor.set(volts);
  }
}
