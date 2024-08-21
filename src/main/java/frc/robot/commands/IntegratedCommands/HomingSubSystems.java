// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntegratedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.commands.ArmCommands.HomingArm;
import frc.robot.commands.ElevatorCommands.HomingElevator;
import frc.robot.commands.intakeCommands.IntakeSetPower;
import frc.robot.subsystems.Arm;

/** Add your docs here. */
public class HomingSubSystems extends ParallelCommandGroup {
    public HomingSubSystems(Elevator elevator, Arm arm, Intake intake) {
        addCommands(new HomingArm(arm), new HomingElevator(elevator), new IntakeSetPower(intake, 0));
    }

}
