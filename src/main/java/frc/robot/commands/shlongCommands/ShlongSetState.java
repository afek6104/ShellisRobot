// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shlongCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shlong;

public class ShlongSetState extends InstantCommand {
  /** Creates a new ShlongSetState. */
  private Shlong m_shlong;
  private ShlongState m_state;
  public ShlongSetState(Shlong shlong, ShlongState state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shlong);
    m_shlong = shlong;
    m_state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shlong.setPosition(m_state.getPosition());
  }
}
