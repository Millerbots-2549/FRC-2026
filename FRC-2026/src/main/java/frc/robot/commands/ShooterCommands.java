// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.shooter.ShooterConstants.FEED_VELOCITY;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_AGITATE_VELOCITY;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ShooterCommands {
  /**
   * Aims the shooter towards a specific target
   *
   * @param shooter The shooter subsystem
   * @param distanceSupplier A supplier giving the distance of the robot from the target
   * @return A command to aim the shooter
   */
  public static Command homeShooter(Shooter shooter, DoubleSupplier distanceSupplier) {
    return Commands.run(
        () -> shooter.applyAutoAimDistance(distanceSupplier.getAsDouble()), shooter);
  }

  /**
   * Shoots balls according to the distance from the hub. Should be coupled with the point at hub
   * command on the driver controller.
   *
   * @param shooter The shooter subsystem
   * @param indexer The indexer subsystem
   * @param distanceSupplier A supplier giving the distance of the robot from the target
   * @return A command to shoot balls.
   */
  public static Command shootWithHoming(
      Shooter shooter, Indexer indexer, DoubleSupplier distanceSupplier) {
    return new SequentialCommandGroup(
        // Gets rid of fuel stuck in shooter
        new ParallelCommandGroup(
                Commands.run(() -> {
                    shooter.setVelocity(FLYWHEEL_AGITATE_VELOCITY);
                    shooter.setFeederVelocity(-100);
                }, shooter))
            .withTimeout(0.75),
        // Spins shooter up to speed, keeping fuel away while it gets up to speed
        new ParallelCommandGroup(
                Commands.run(() -> {
                    shooter.applyAutoAimDistance(distanceSupplier.getAsDouble());
                    shooter.setFeederVelocity(0);
                }, shooter))
            .withDeadline(Commands.waitUntil(shooter::shooterAtSpeed)),
        // Runs indexer forward once shooter is at speed
        new ParallelCommandGroup(
            Commands.run(() -> {
                shooter.applyAutoAimDistance(distanceSupplier.getAsDouble());
                shooter.setFeederVelocity(FEED_VELOCITY);
            }, shooter),
            Commands.run(() -> indexer.setRollerSpeed(1400), indexer)));
  }
}
