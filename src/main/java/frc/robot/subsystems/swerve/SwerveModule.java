package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
    public CANcoderConfiguration m_AbsuloteEncoderconfiguration;


    public SwerveModule(int IDdrive, int IDstirring, int IDcanCoder){
        m_drive = new TalonFX(IDdrive);
        m_steering = new TalonFX(IDstirring);
        m_AbsuloteEncoder = new CANcoder(IDcanCoder);
        m_PositionVoltage = new PositionVoltage(0);
        m_DutyCycleOut = new DutyCycleOut(0);
        m_steeringconfiguration = new TalonFXConfiguration();
        m_driveconfiguration = new TalonFXConfiguration();
        m_AbsuloteEncoderconfiguration = new CANcoderConfiguration();

        m_steeringconfiguration.Slot0.withKP(Constants.SwerveModule.P);

        m_AbsuloteEncoderconfiguration.MagnetSensor.SensorDirection = Constants.SwerveModule.cancoderInvert;

        m_steeringconfiguration.Feedback.withSensorToMechanismRatio(Constants.SwerveModule.steeringGearRatio);
        m_driveconfiguration.Feedback.withSensorToMechanismRatio(Constants.SwerveModule.driveGearRatio);
        

        m_steeringconfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
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

    Rotation2d offset = Rotation2d.fromDegrees(-136.845703125);
    public void resetAbsulte(){
        m_steering.setPosition(getAbsolutePosition().minus(offset).getRotations());
    }

    public Rotation2d getAbsolutePosition(){
        return Rotation2d.fromRotations(m_AbsuloteEncoder.getAbsolutePosition().getValue());
    }

    public Rotation2d getSteeringAngle(){
        return Rotation2d.fromRotations(m_steering.getPosition().getValue());
    }
    
    public void steeringSetPosition(Rotation2d position){
        m_steering.setControl(m_PositionVoltage.withPosition((position.getRotations())));
    }

    double maxVelocity = 4.7;
    public void driveSetPower(double velocity){
        m_drive.setControl(m_DutyCycleOut.withOutput(velocity/maxVelocity));
    }
    
    public void setModuleState(Rotation2d position, double velocity){
        steeringSetPosition(position);
        driveSetPower(velocity);
    }

    public void disableMotors(){
        m_drive.disable();
        m_steering.disable();
    }
} 