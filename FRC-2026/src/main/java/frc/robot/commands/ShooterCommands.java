// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ShooterCommands {
  public static Command homeShooter(Shooter shooter, DoubleSupplier distanceSupplier) {
    return Commands.run(() -> {
      shooter.applyAutoAimDistance(distanceSupplier.getAsDouble());
    }, shooter);
  }
}
