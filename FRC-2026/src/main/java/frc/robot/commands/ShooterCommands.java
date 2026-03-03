// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ShooterCommands {
  public static Command homeShooter(Shooter shooter, DoubleSupplier distanceSupplier) {
    return Commands.run(
        () -> {
          shooter.applyAutoAimDistance(distanceSupplier.getAsDouble());
        },
        shooter);
  }

  public static Command shootWithHoming(
      Shooter shooter, Indexer indexer, DoubleSupplier distanceSupplier) {
    return new SequentialCommandGroup(
        Commands.run(
                () -> {
                  indexer.setRollerSpeed(-1200);
                  shooter.setVelocity(1000);
                },
                indexer,
                shooter)
            .withTimeout(0.5),
        Commands.run(
                () -> {
                  indexer.setRollerSpeed(0);
                  shooter.setVelocity(-2810);
                },
                indexer,
                shooter)
            .withDeadline(Commands.waitUntil(() -> shooter.shooterAtSpeed())),
        Commands.run(
            () -> {
              indexer.setRollerSpeed(-800);
              shooter.setVelocity(-2810);
            },
            shooter,
            indexer));

    /*
    return Commands.run(
        () -> {
          switch (stage) {
            case PURGING:
              indexer.setRollerSpeed(-1200);
              shooter.setVelocity(1000);
              if (shooter.shooterAtSpeed()) {
                stage = SHOOTING_STAGE.SPINNING_UP;
              }
              break;

            case SPINNING_UP:
              indexer.setRollerSpeed(-800);
              shooter.applyAutoAimDistance(distanceSupplier.getAsDouble());
              if (shooter.shooterAtSpeed()) {
                stage = SHOOTING_STAGE.FEEDING;
              }
              break;

            default:
              indexer.setRollerSpeed(800);
              shooter.applyAutoAimDistance(distanceSupplier.getAsDouble());
              break;
          }
        },
        shooter,
        indexer);*/
  }
}
