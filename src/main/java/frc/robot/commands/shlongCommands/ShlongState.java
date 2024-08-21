// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shlongCommands;

/** Add your docs here. */
public enum ShlongState {
    IN(0.0),
    OUT(0.4);
    
    private double position;
    ShlongState(double position){
        this.position = position;
    }

    public double getPosition(){
        return position;
    }
};
