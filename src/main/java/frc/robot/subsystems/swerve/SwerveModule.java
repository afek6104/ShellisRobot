package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class SwerveModule {
    
    private TalonFX m_drive;
    private TalonFX m_steering;
    private CANcoder m_AbsuloteEncoder;
    private PositionVoltage m_PositionVoltage;
    private DutyCycleOut m_DutyCycleOut;
    private TalonFXConfiguration m_steeringconfiguration;
    private TalonFXConfiguration m_driveconfiguration;


    public SwerveModule(int IDdrive, int IDstirring, int IDcanCoder){
        m_drive = new TalonFX(IDdrive);
        m_steering = new TalonFX(IDstirring);
        m_AbsuloteEncoder = new CANcoder(IDcanCoder);
        m_PositionVoltage = new PositionVoltage(0);
        m_DutyCycleOut = new DutyCycleOut(0);
        m_steeringconfiguration = new TalonFXConfiguration();
        m_driveconfiguration = new TalonFXConfiguration();

        m_steeringconfiguration.Slot0.withKP(Constants.SwerveModule.P);

        m_steeringconfiguration.Feedback.withSensorToMechanismRatio(150/7);
        m_driveconfiguration.Feedback.withSensorToMechanismRatio(6.75);
        

        m_steeringconfiguration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        m_driveconfiguration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        m_steeringconfiguration.withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(Constants.SwerveModule.CurrentLimits)
        .withSupplyCurrentLimitEnable(Constants.SwerveModule.currentLimitEnbale) 
        .withSupplyCurrentThreshold(Constants.SwerveModule.CurrentLimitsThreshold)
        .withSupplyTimeThreshold(Constants.SwerveModule.TimeThreshold));

        m_driveconfiguration.withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(Constants.SwerveModule.CurrentLimits)
        .withSupplyCurrentLimitEnable(Constants.SwerveModule.currentLimitEnbale) 
        .withSupplyCurrentThreshold(Constants.SwerveModule.CurrentLimitsThreshold)
        .withSupplyTimeThreshold(Constants.SwerveModule.TimeThreshold));

        m_drive.getConfigurator().apply(m_driveconfiguration);
        m_steering.getConfigurator().apply(m_steeringconfiguration);

        resetAbsulte();
    }

    double offset = 0.374267578125;
    public void resetAbsulte(){
        m_steering.setPosition(m_AbsuloteEncoder.getAbsolutePosition().getValue() - offset);
    }

    public double getAbsolutePosition(){
        return Rotation2d.fromRotations(m_AbsuloteEncoder.getAbsolutePosition().getValue()).getDegrees();
    }

    public double getSteeringAngle(){
        return Rotation2d.fromRotations(steeringRotationsToUnits(m_steering.getPosition().getValue())).getDegrees();
    }
    
    public void steeringSetPosition(double position){
        m_steering.setControl(m_PositionVoltage.withPosition(steeringUnitsToRotations(position)));
    }

    double maxVelocity = 4.7;
    public void driveSetPower(double velocity){
        m_drive.setControl(m_DutyCycleOut.withOutput(velocity/maxVelocity));
    }
    
    public void setModuleState(double position, double velocity){
        steeringSetPosition(position);
        driveSetPower(velocity);
    }

    private double driveRotationsToUnits(double rotation){
        return rotation * 0.0363728 * Math.PI;
    }
    
    private double driveUnitsToRotations(double meter){
        return meter / (0.0363728 * Math.PI);
    }

    private double steeringRotationsToUnits(double rotation){
        return rotation * 360;
      }
    
      private double steeringUnitsToRotations(double angle){
        return angle / 360;
      }

    public void disableMotors(){
        m_drive.disable();
        m_steering.disable();
    }
} 