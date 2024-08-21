// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private static Elevator m_instance = null;

  private TalonFX m_motorMaster; 
  private TalonFX m_motorFollower;
  private TalonFXConfiguration configuration;
  private DutyCycleOut m_DutyCycleOut;
  private PositionVoltage m_PositionVoltage;
  private StatusSignal<Double> m_positionSignal;

  private Elevator() {
    m_motorMaster = new TalonFX(Constants.Elevator.MasterID);
    m_motorMaster.getConfigurator().apply(new TalonFXConfiguration());
    m_motorFollower = new TalonFX(Constants.Elevator.FollowerID);
    m_motorFollower.getConfigurator().apply(new TalonFXConfiguration());

    m_DutyCycleOut = new DutyCycleOut(0);
    m_PositionVoltage = new PositionVoltage(0);
    configuration = new TalonFXConfiguration();

    m_positionSignal = m_motorMaster.getPosition();

    configuration.withCurrentLimits(new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(Constants.Elevator.CurrentLimits)
    .withSupplyCurrentLimitEnable(Constants.Elevator.currentLimitEnbale) 
    .withSupplyCurrentThreshold(Constants.Elevator.CurrentLimitsThreshold)
    .withSupplyTimeThreshold(Constants.Elevator.TimeThreshold));

    configuration.Slot0
    .withKP(Constants.Elevator.P)
    .withKI(Constants.Elevator.I)
    .withKD(Constants.Elevator.D)
    .withKS(Constants.Elevator.S)
    .withKV(Constants.Elevator.V)
    .withKG(Constants.Elevator.G)
    .withGravityType(GravityTypeValue.Elevator_Static);

    configuration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    configuration.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(Constants.Elevator.ForwardSoftLimitEnable)
    .withForwardSoftLimitThreshold(UnitsToRotations(Constants.Elevator.ForwardSoftLimit))
    .withReverseSoftLimitEnable(Constants.Elevator.ReverseSoftLimitEnable)
    .withReverseSoftLimitThreshold(UnitsToRotations(Constants.Elevator.ReverseSoftLimit)));

    configuration.Feedback.withSensorToMechanismRatio(12.1693121693);

    m_motorFollower.setControl(new Follower(m_motorMaster.getDeviceID(), Constants.Elevator.opposeMaster)); 

    m_motorMaster.getConfigurator().apply(configuration);
    m_motorFollower.getConfigurator().apply(configuration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_positionSignal.refresh();
    SmartDashboard.putNumber("elevator Position", getPosition());
  }

  public void setPower(double power){
    m_motorMaster.setControl(m_DutyCycleOut.withOutput(power));
    }

  public void disableMotors(){
    m_motorMaster.disable();
    m_motorFollower.disable();
  }

  public void setPosition(double position){
     m_motorMaster.setControl(m_PositionVoltage.withPosition(UnitsToRotations(position)));
  }

  public double getPosition(){
    return RotationsToUnits(m_positionSignal.getValueAsDouble());
  }

  public void resetPosition(double position){
    m_motorMaster.setPosition(position);
    m_motorFollower.setPosition(position);
  }

  private double RotationsToUnits(double rotation){
    return rotation * 0.0363728 * Math.PI;
  }

  private double UnitsToRotations(double meter){
    return meter / (0.0363728 * Math.PI);
  }

  public double getCurrent(){
    return m_motorMaster.getSupplyCurrent().getValueAsDouble();
  }
  
  public void softLimitEnable(boolean enable){
    m_motorMaster.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(enable)
    .withReverseSoftLimitEnable(enable));
  }

  public static Elevator getInstance(){
    if(m_instance == null){
      return new Elevator();
    }
    return m_instance;
  }
}
