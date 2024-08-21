// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntegratedCommands;

import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ArmState;
import frc.robot.commands.ElevatorCommands.ElevatorState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class AmpThrowUp extends RobotSetStateSequential{
    public AmpThrowUp(Elevator elevator, Arm arm, Intake intake) {
        super(elevator, ElevatorState.amp, arm, ArmState.amp, intake, Constants.Intake.intakeSpeed);
    }    
}
