// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class HomingElevator extends Command {
  /** Creates a new HomingElevator. */
  private Elevator m_Elevator;
  public HomingElevator(Elevator elevator) {
    addRequirements(elevator);
    m_Elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elevator.softLimitEnable(false);
    m_Elevator.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Elevator.getPosition() < 0.03){
      m_Elevator.setPower(-0.05);
    }
    SmartDashboard.putNumber("ElevatorMotorCurrent", m_Elevator.getCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.softLimitEnable(true);
    m_Elevator.setPosition(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.getCurrent() > 1.25 && m_Elevator.getPosition() < 0.03;
  }
}
