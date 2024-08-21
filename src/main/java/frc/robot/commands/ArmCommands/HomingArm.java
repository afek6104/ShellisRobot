// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class HomingArm extends Command {
  /** Creates a new HomingArm. */
  private Arm m_arm;
  public HomingArm(Arm arm) {
    addRequirements(arm);
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.softLimitEnable(false);
    m_arm.setPosition(127);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.getPosition() > 90){
      m_arm.setPower(0.1);
    }
    SmartDashboard.putNumber("ArmMotorCurrent", m_arm.getCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.softLimitEnable(true);
    m_arm.resetPosition(127);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getCurrent() > 4.5 && m_arm.getPosition() > 115;
  }
}
