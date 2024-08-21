// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private static Arm m_instance = null;

  private TalonFX m_motorMaster;
  private TalonFXConfiguration configuration;
  private PositionVoltage m_PositionVoltage;
  private DutyCycleOut m_DutyCycleOut;
  private StatusSignal<Double> m_positionSignal;

  public Arm() {
    m_motorMaster = new TalonFX(Constants.Arm.MasterID);
    m_motorMaster.getConfigurator().apply(new TalonFXConfiguration());

    m_PositionVoltage = new PositionVoltage(0);
    m_DutyCycleOut = new DutyCycleOut(0);
    configuration = new TalonFXConfiguration();

    m_positionSignal = m_motorMaster.getPosition();

    configuration.withCurrentLimits(new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(Constants.Arm.CurrentLimits)
    .withSupplyCurrentLimitEnable(Constants.Arm.currentLimitEnbale) 
    .withSupplyCurrentThreshold(Constants.Arm.CurrentLimitsThreshold)
    .withSupplyTimeThreshold(Constants.Arm.TimeThreshold));

    configuration.Feedback.withSensorToMechanismRatio(22.857);

    configuration.Slot0
    .withKP(Constants.Arm.P)
    .withKI(Constants.Arm.I)
    .withKD(Constants.Arm.D)
    .withKS(Constants.Arm.S)
    .withKV(Constants.Arm.V)
    .withKG(Constants.Arm.G)
    .withGravityType(GravityTypeValue.Arm_Cosine);

    configuration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    configuration.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(Constants.Arm.ForwardSoftLimitEnable)
    .withForwardSoftLimitThreshold(UnitsToRotations(Constants.Arm.ForwardSoftLimit))
    .withReverseSoftLimitEnable(Constants.Arm.ReverseSoftLimitEnable)
    .withReverseSoftLimitThreshold(UnitsToRotations(Constants.Arm.ReverseSoftLimit)));

    m_motorMaster.getConfigurator().apply(configuration);

    // SmartDashboard.putBoolean("setPID", false);
    // SmartDashboard.putNumber("Kp", 0);
    // SmartDashboard.putNumber("Ki", 0);
    // SmartDashboard.putNumber("Kd", 0);
    // SmartDashboard.putNumber("Kg", 0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(SmartDashboard.getBoolean("setPID", false)){
    //   m_motorMaster.getConfigurator().apply(new Slot0Configs()
    //   .withKP(SmartDashboard.getNumber("Kp", 0))
    //   .withKI(SmartDashboard.getNumber("ki", 0))
    //   .withKD(SmartDashboard.getNumber("Kp", 0))
    //   .withKG(SmartDashboard.getNumber("Kg", 0)));
    // }
    m_positionSignal.refresh();
    SmartDashboard.putNumber("arm Position", getPosition());
  }

  public void disableMotors(){
    m_motorMaster.disable();
  }

  public void setPosition(double position){
     m_motorMaster.setControl(m_PositionVoltage.withPosition(UnitsToRotations(position)));
  }

  public double getPosition(){
    return RotationsToUnits(m_positionSignal.getValueAsDouble());
  }

  public void resetPosition(double position){
    m_motorMaster.setPosition(UnitsToRotations(position));
  }

  private double RotationsToUnits(double rotation){
    return rotation * 360;
  }

  private double UnitsToRotations(double angle){
    return angle / 360;
  }

  public double getCurrent(){
    return m_motorMaster.getSupplyCurrent().getValueAsDouble();
  }

  public void softLimitEnable(boolean enable){
    m_motorMaster.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(enable)
    .withReverseSoftLimitEnable(enable));
  }

  public void setPower(double power) {
    m_motorMaster.set(power);
  }

  public static Arm getInstance(){
    if(m_instance == null){
      return new Arm();
    }
    return m_instance;
  }
}
