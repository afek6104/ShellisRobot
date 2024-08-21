// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ElevatorSetState extends InstantCommand {
  private Elevator m_elevator;
  private ElevatorState m_state;
  public ElevatorSetState(Elevator elevator, ElevatorState state) {
    addRequirements(elevator);
    m_elevator = elevator;
    m_state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setPosition(m_state.getPosition());
  }
}
