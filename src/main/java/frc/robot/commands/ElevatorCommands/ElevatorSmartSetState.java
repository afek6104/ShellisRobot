// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorSmartSetState extends Command {
  /** Creates a new ElevatorSmartSetState. */
  Elevator m_elevator;
  ElevatorState m_state;
  public ElevatorSmartSetState(Elevator elevator, ElevatorState state) {
    addRequirements(elevator);
    m_elevator = elevator;
    m_state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setPosition(m_state.getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_state.getPosition() - m_elevator.getPosition()) < Constants.Elevator.allowAbleError;
  }
}
