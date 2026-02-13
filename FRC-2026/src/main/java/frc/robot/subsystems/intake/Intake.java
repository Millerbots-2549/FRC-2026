// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public final IntakePivotIO pivotIO;
  public final IntakeRollerIO rollerIO;

  private final IntakePivotIOInputsAutoLogged pivotIOInputs = new IntakePivotIOInputsAutoLogged();
  private final IntakeRollerIOInputsAutoLogged rollerIOInputs =
      new IntakeRollerIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake(IntakePivotIO pivotIO, IntakeRollerIO rollerIO) {
    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotIOInputs);
    rollerIO.updateInputs(rollerIOInputs);

    pivotIO.periodic();
  }

  /**
   * Resets the position of the intake, given that it is currently at its lowest possible position.
   */
  public void zeroIntakeGround() {}

  /**
   * Sets the angle of the intake arm
   *
   * @param angle The setpoint
   */
  public void setIntakeAngle(Rotation2d angle) {
    this.pivotIO.setPivotAngle(angle);
  }

  /**
   * Sets the angle of the intake arm
   *
   * @param angle The setpoint
   */
  public void setIntakeAngle(double angle) {
    this.setIntakeAngle(Rotation2d.fromRadians(angle));
  }

  /**
   * Sets the speed of the intake rollers
   *
   * @param speed The speed in rpm
   */
  public void setRollerSpeed(double speed) {
    this.rollerIO.setRollerOpenLoop(speed);
  }
}
