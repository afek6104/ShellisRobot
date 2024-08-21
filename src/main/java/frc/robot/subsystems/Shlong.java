// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shlong extends SubsystemBase {
  private CANSparkFlex m_motor;
  private SparkPIDController m_pidController;

  public Shlong(int motorID, boolean invert) {
    m_motor = new CANSparkFlex(motorID, MotorType.kBrushless); 
    m_pidController = m_motor.getPIDController();
    m_pidController.setP(Constants.shlong.P);
    m_pidController.setI(Constants.shlong.I);
    m_pidController.setD(Constants.shlong.D);
    m_motor.getEncoder().setPositionConversionFactor(Constants.shlong.SproketCircumfrence); 
    m_motor.setInverted(invert);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.shlong.kSoftLimitForward);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.shlong.kSoftLimitReverse);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.burnFlash();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(double position){
    m_pidController.setReference(position, ControlType.kPosition);
  }
  
  public double getPosition(){
    return m_motor.getEncoder().getPosition();
  }

  public void disableMotor(){
    m_motor.disable();
  }
}
