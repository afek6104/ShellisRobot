// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

/** Add your docs here. */
public enum ElevatorState {
    floor(0.0),
    def(0.2),
    amp(0.85);
    
    private double position;
    ElevatorState(double position){
        this.position = position;
    }

    public double getPosition(){
        return position;
    }
};
