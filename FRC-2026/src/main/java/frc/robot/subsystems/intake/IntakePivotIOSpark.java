// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_GEAR_RATIO;
import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_KCOS;
import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_KD;
import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_KI;
import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_KP;
import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_MAX_ACCELERATION;
import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_MAX_VELOCITY;
import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_MOTOR_ID;
import static frc.robot.util.PhoenixUtil.ifOk;
import static frc.robot.util.PhoenixUtil.sparkStickyFault;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
@SuppressWarnings("unused")
public class IntakePivotIOSpark implements IntakePivotIO {
  // Objects for Spark Max
  protected final SparkMax pivotMotor;
  protected final RelativeEncoder pivotEncoder;
  protected final SparkMaxConfig pivotConfig;
  private final ProfiledPIDController pivotController;

  /** The setpoint for the intake arm in degrees */
  private double pivotSetpoint;

  public IntakePivotIOSpark() {
    pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

    pivotConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake);

    pivotConfig.absoluteEncoder.positionConversionFactor(PIVOT_GEAR_RATIO);
    pivotConfig.absoluteEncoder.inverted(false);

    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = pivotMotor.getEncoder();

    this.pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tryUntilOk(pivotMotor, 5, () -> pivotEncoder.setPosition(0.0));

    pivotController = new ProfiledPIDController(PIVOT_KP, PIVOT_KI, PIVOT_KD,
        new TrapezoidProfile.Constraints(PIVOT_MAX_VELOCITY, PIVOT_MAX_ACCELERATION));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(pivotMotor, pivotEncoder::getVelocity, (value) -> inputs.pivotVelocity = value);
    ifOk(
        pivotMotor,
        new DoubleSupplier[] {pivotMotor::getAppliedOutput, pivotMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);

    inputs.currentAngle = pivotEncoder.getPosition();
    inputs.currentSetpoint = pivotSetpoint;
  }

  private double lastAngle = 0;

  @Override
  public void setPivotAngle(Rotation2d target) {
    if (target.getDegrees() != lastAngle) {
      pivotSetpoint = target.getDegrees();
      lastAngle = target.getDegrees();
    }
  }

  @Override
  public void setPivotOpenLoop(double output) {
    pivotMotor.set(output);
  }

  @Override
  public void resetAngle(double resetAngleDeg) {
    tryUntilOk(pivotMotor, 5, () -> pivotEncoder.setPosition(resetAngleDeg));
  }

  @Override
  public void periodic() {
    double output = pivotController.calculate(pivotEncoder.getPosition(), pivotSetpoint);
    output += Math.sin(Math.toRadians(pivotEncoder.getPosition())) * PIVOT_KCOS;

    pivotMotor.set(output);
  }
}
