// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;
/** Add your docs here. */
public enum ArmState {
    
    floor(-10),
    in(120),
    amp(0.85);
    
    private double position;
    ArmState(double position){
        this.position = position;
    }

    public double getPosition(){
        return position;
    }
};
