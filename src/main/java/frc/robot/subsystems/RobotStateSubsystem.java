// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RobotStateSubsystem extends SubsystemBase {
  /** Creates a new RobotStateSubSystem. */
  public static Elevator m_elevator = Elevator.getInstance();
  public static Arm m_arm = Arm.getInstance();
  public static Intake m_intake = Intake.getInstance();
  public static Shlong m_leftShlong = new Shlong(Constants.shlong.leftMotorId, true);
  public static Shlong m_RightShlong = new Shlong(Constants.shlong.rightMotorId, false);
  public RobotStateSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void disableSubsystems(){
    m_elevator.disableMotors();
    m_arm.disableMotors();
    m_intake.disableMotor();
    m_RightShlong.disableMotor();
    m_leftShlong.disableMotor();
  }
}
