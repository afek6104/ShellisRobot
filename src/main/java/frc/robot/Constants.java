// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Intake{
    public static final int motorId = 25;
    public static final int kCurrentLimit = 40;
    public static final double intakeSpeed = 0.5;
    public static final double throwUpSpped = -0.5;
  }

  public static class shlong{
    public static final int leftMotorId = 26;
    public static final int rightMotorId = 27;
    public static final float kSoftLimitForward = 50;
    public static final float kSoftLimitReverse = -50;
    public static final double SproketCircumfrence = 0.0398272 * Math.PI;
    public static final double P = 0.65;
    public static final double I = 0;
    public static final double D = 0;
  }

  public static class Elevator{
    public static final int MasterID = 5;
    public static final int FollowerID = 6;
    public static final double P = 2.5;
    public static final double I = 0.1;
    public static final double D = 0.5;
    public static final double S = 0.05;
    public static final double V = 0;
    public static final double G = 0.12;
    public static final boolean currentLimitEnbale = true;
    public static final double CurrentLimits = 40;
    public static final double CurrentLimitsThreshold = 60;
    public static final double TimeThreshold = 0.1;
    public static final boolean ForwardSoftLimitEnable = false;
    public static final boolean ReverseSoftLimitEnable = false;
    public static final boolean opposeMaster = false;
    public static final double ForwardSoftLimit = 0.85;
    public static final double ReverseSoftLimit = 0.0;
    public static final double allowAbleError = 0.01;
  }
  
  public static class Arm{
    public static final int MasterID = 8;
    public static final double P = 12;
    public static final double I = 0;
    public static final double D = 0;
    public static final double S = 0;
    public static final double V = 0;
    public static final double G = 0.35;
    public static final double CurrentLimits = 40;
    public static final boolean currentLimitEnbale = true;
    public static final double CurrentLimitsThreshold = 60;
    public static final double TimeThreshold = 0.1;
    public static final boolean ForwardSoftLimitEnable = true;
    public static final boolean ReverseSoftLimitEnable = true;
    public static final double ForwardSoftLimit = 127;
    public static final double ReverseSoftLimit = -15;
    public static final double allowAbleError = 1;

  }

  public static class SwerveModule {
    public static final double P = 100;
    public static final double CurrentLimits = 40;
    public static final boolean currentLimitEnbale = true;
    public static final double CurrentLimitsThreshold = 60;
    public static final double TimeThreshold = 0.1; 
  }
}
