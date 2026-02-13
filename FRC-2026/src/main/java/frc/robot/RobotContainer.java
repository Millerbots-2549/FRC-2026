// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.intake.IntakeConstants.ROLLER_MAX_SPEED;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SimulationConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivotIO;
import frc.robot.subsystems.intake.IntakePivotIOSpark;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOSpark;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.HoodIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Simulation
  public SwerveDriveSimulation swerveDriveSimulation;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveDriveSimulation = null;
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intake =
            new Intake(
                new IntakePivotIOSpark(),
                new IntakeRollerIOSpark(new SparkMaxConfig()));
        shooter =
            new Shooter(
                new HoodIOTalonFX(),
                new FlywheelIOTalonFX());
        break;

      case SIM:
        /* Create a swerve drive simulation */
        this.swerveDriveSimulation =
            new SwerveDriveSimulation(
                // Specify Configuration
                SimulationConstants.driveTrainSimulationConfig,
                // Specify starting pose
                new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(swerveDriveSimulation.getGyroSimulation()) {},
                new ModuleIOTalonSim(swerveDriveSimulation.getModules()[0]),
                new ModuleIOTalonSim(swerveDriveSimulation.getModules()[1]),
                new ModuleIOTalonSim(swerveDriveSimulation.getModules()[2]),
                new ModuleIOTalonSim(swerveDriveSimulation.getModules()[3]));
        intake = 
            new Intake(
                new IntakePivotIO() {},
                new IntakeRollerIO() {});
        shooter =
            new Shooter(
                new HoodIO() {},
                new FlywheelIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = 
            new Intake(
                new IntakePivotIO() {},
                new IntakeRollerIO() {});
        shooter =
            new Shooter(
                new HoodIO() {},
                new FlywheelIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    controller
        .y()
        .whileTrue(
            DriveCommands.joystickDrivePointingTowardsWithShooter(
                drive,
                shooter,
                () -> -controller.getLeftX(),
                () -> -controller.getLeftY(),
                () -> drive.getPose()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            Constants.currentMode == Mode.REAL
                                ? new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)
                                : swerveDriveSimulation.getSimulatedDriveTrainPose()),
                    drive)
                .ignoringDisable(true));

    intake.setDefaultCommand(Commands.none());

    controller.leftBumper().onTrue(Commands.run(() -> intake.setIntakeAngle(Rotation2d.fromDegrees(20)), intake));
    controller.rightBumper().onTrue(Commands.run(() -> intake.setIntakeAngle(Rotation2d.fromDegrees(0)), intake));
    controller.leftTrigger().whileTrue(Commands.run(() -> intake.setRollerSpeed(ROLLER_MAX_SPEED * controller.getLeftTriggerAxis()), intake));
    controller.rightTrigger().whileTrue(Commands.run(() -> intake.setRollerSpeed(-ROLLER_MAX_SPEED * controller.getRightTriggerAxis()), intake));

    shooter.setDefaultCommand(Commands.run(() -> shooter.sustainHood(), shooter));

    controller.povUp().whileTrue(Commands.run(() -> shooter.raiseHood(), shooter));
    controller.povDown().whileTrue(Commands.run(() -> shooter.lowerHood(), shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
