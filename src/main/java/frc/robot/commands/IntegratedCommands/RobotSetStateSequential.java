// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntegratedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.commands.ArmCommands.ArmSmartSetState;
import frc.robot.commands.ArmCommands.ArmState;
import frc.robot.commands.ElevatorCommands.ElevatorSmartSetState;
import frc.robot.commands.ElevatorCommands.ElevatorState;
import frc.robot.commands.intakeCommands.IntakeSetPower;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RobotSetStateSequential extends SequentialCommandGroup {
  /** Creates a new SetRobotState. */
  private Elevator m_elevator;
  private ElevatorState m_stateElevator;
  private Arm m_arm;
  private ArmState m_stateArm;
  private double m_power;
  private Intake m_intake;
  public RobotSetStateSequential(Elevator elevator, ElevatorState stateElevator, Arm arm, ArmState stateArm, Intake intake, double power) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_elevator = elevator;
    m_stateElevator = stateElevator;
    m_arm = arm;
    m_stateArm = stateArm;
    m_power = power;
    m_intake = intake;
    addCommands(new ElevatorSmartSetState(m_elevator, m_stateElevator),new ArmSmartSetState(m_arm, m_stateArm), new IntakeSetPower(m_intake, m_power));
  }
}
