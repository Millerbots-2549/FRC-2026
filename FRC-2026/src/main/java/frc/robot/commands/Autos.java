// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

/** Add your docs here. */
public class Autos {
  public static final Command shootAllBallsFromStartPos(
      Drive drive, Shooter shooter, Indexer indexer) {
    return new ParallelCommandGroup(
        DriveCommands.joystickDrivePointingTowards(
            drive,
            () -> 0.0,
            () -> 0.0,
            () -> new Pose2d(Constants.HUB_TRANSLATION, Rotation2d.kZero)),
        ShooterCommands.shootWithHoming(
            shooter,
            indexer,
            () -> drive.getPose().getTranslation().getDistance(Constants.HUB_TRANSLATION)));
  }

  public static Command shootFromHub(Shooter shooter, Indexer indexer) {
    return new SequentialCommandGroup(
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
                  indexer.setRollerSpeed(800);
                  shooter.setVelocity(-2810);
                },
                shooter,
                indexer)
            .withTimeout(4));
  }
}
