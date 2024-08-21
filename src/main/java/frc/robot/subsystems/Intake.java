// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake m_instance = null;
  private CANSparkFlex m_motor;
  
  private Intake() {
    m_motor = new CANSparkFlex(Constants.Intake.motorId, MotorType.kBrushless);
    m_motor.setInverted(true);
    m_motor.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power){
    m_motor.set(power);
  }

  public void disableMotor(){
    m_motor.disable();
  }

  public static Intake getInstance(){
    if(m_instance == null){
      m_instance = new Intake();
    }
    return m_instance;
  }
}
