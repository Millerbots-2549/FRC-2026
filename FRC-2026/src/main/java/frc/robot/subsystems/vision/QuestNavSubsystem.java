// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  private QuestNav questNav = new QuestNav();

  /** Creates a new QUestNavSubsystem. */
  public QuestNavSubsystem() {}

  @Override
  public void periodic() {
    questNav.commandPeriodic();
  }
}
