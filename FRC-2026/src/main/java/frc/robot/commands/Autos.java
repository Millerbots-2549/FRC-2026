// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_MAX_DEGREES;
import static frc.robot.subsystems.intake.IntakeConstants.ROLLER_MAX_SPEED;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/** Add your docs here. */
public class Autos {
  public static final PathConstraints FAST_PATH_CONSTRAINTS = new PathConstraints(0.0, 0.0, 0.0, 0.0);

  /**
   * Shoots every pre-loaded ball from the robot's starting position
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param indexer Indexer Subsystem
   * @return The Auto to shoot preloaded balls
   */
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
            () -> drive.getDistToHub()));
  }

  /**
   * Shoots all preloaded balls from against the hub. DOnt use this, its bad.
   * @param shooter
   * @param indexer
   * @return
   */
  public static Command shootFromHub(Shooter shooter, Indexer indexer) {
    return new SequentialCommandGroup(
        Commands.run(
                () -> {
                  indexer.setRollerSpeed(0);
                  shooter.setVelocity(-2810);
                  shooter.applyHoodSetpoint(Rotation2d.fromDegrees(5));
                },
                indexer,
                shooter)
            .withDeadline(Commands.waitUntil(() -> shooter.shooterAtSpeed())),
        Commands.run(
                () -> {
                  indexer.setRollerSpeed(800);
                  shooter.setVelocity(-2810);
                  shooter.applyHoodSetpoint(Rotation2d.fromDegrees(5));
                },
                shooter,
                indexer)
            .withTimeout(4));
  }

  /**
   * An auto for running a single cycle, as well as shooting all preloaded balls.
   * 
   * <p>This auto will:
   * <ul>
   *    <li>Shoot all preloaded balls
   *    <li>Drive to the neutral zone and pick up balls
   *    <li>Drive back to the hub, and shoot all loaded balls.
   * </ul>
   * <p>This auto can start from any position.
   * @param shooter
   * @param indexer
   * @param intake
   * @param drive
   * @return
   */
  public static Command singleCycleAuto(Shooter shooter, Indexer indexer, Intake intake, Drive drive) {
    PathPlannerPath toCycleStart;
    try {
      toCycleStart = PathPlannerPath.fromPathFile("Left Cycle Part 1");
    } catch (Exception e) {
      return Commands.none();
    }
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        ShooterCommands.shootWithHoming(shooter, indexer, () -> drive.getDistToHub()),
        DriveCommands.driveToPoseWhileAiming(drive, () -> toCycleStart.getStartingDifferentialPose().getTranslation(), () -> Constants.HUB_TRANSLATION)
      ),
      new ParallelCommandGroup(
        Commands.run(() -> {
          shooter.setVelocity(0);
          intake.setIntakeAngle(Rotation2d.fromDegrees(PIVOT_MAX_DEGREES), false);
          intake.setRollerSpeed(-ROLLER_MAX_SPEED);
        }, shooter),
        AutoBuilder.followPath(toCycleStart)
      ),
      new ParallelCommandGroup(
        ShooterCommands.shootWithHoming(shooter, indexer, () -> drive.getDistToHub()),
        DriveCommands.driveToPoseWhileAiming(drive, () -> Constants.ROBOT_AGAINST_HUB, () -> Constants.HUB_TRANSLATION))
    );
  }
}
