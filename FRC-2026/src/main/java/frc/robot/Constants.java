// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class SimulationConstants {
    // Create and configure a drivetrain simulation configuration
    public static final DriveTrainSimulationConfig driveTrainSimulationConfig =
        DriveTrainSimulationConfig.Default()
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofPigeon2())
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                    DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                    6.12, // Drive motor gear ratio.
                    12.8, // Steer motor gear ratio.
                    Volts.of(0.1), // Drive friction voltage.
                    Volts.of(0.1), // Steer friction voltage
                    Inches.of(2), // Wheel radius
                    KilogramSquareMeters.of(0.03), // Steer MOI
                    1.2)) // Wheel COF
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(Inches.of(30), Inches.of(30));
  }
}
