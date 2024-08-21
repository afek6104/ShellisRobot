// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands.ResetPositionArm;
import frc.robot.commands.ElevatorCommands.ResetPositionElevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

      RobotStateSubsystem robotStateSubsystem = new RobotStateSubsystem();
      Elevator m_elevator = Elevator.getInstance();
      Arm m_arm = Arm.getInstance();
      Intake m_intake = Intake.getInstance();
      SwerveModule m_SwerveModule = new SwerveModule(10, 11, 12);
      Translation2d a = new Translation2d(0, 0);
      ChassisSpeeds b = new ChassisSpeeds();
      SwerveDriveKinematics c = new SwerveDriveKinematics(null);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    c.toSwerveModuleStates(b);
    // m_driverController.triangle().onTrue(new RobotSetStateParallel(m_elevator, ElevatorState.def, m_arm, ArmState.amp, m_intake, 0.5));
    // m_driverController.circle().onTrue(new RobotSetStateParallel(m_elevator, ElevatorState.floor, m_arm, ArmState.in, m_intake, 0));
    // m_driverController.R1().onTrue(new HomingSubSystems(m_elevator, m_arm, m_intake));
    // m_driverController.R2().onTrue(new RobotSetStateParallel(m_elevator, ElevatorState.amp, m_arm, ArmState.amp, m_intake, 0));
    // m_driverController.R2().debounce(2).onTrue(new RobotSetStateParallel(m_elevator, ElevatorState.amp, m_arm, ArmState.amp, m_intake, Constants.Intake.throwUpSpped));
    // m_driverController.L2().whileTrue(new FloorPIckUp(m_elevator, m_arm, m_intake));
    // m_driverController.square().onTrue(new InstantCommand(() -> m_arm.disableMotors()));
    // m_driverController.square().onTrue(new InstantCommand(() -> m_elevator.disableMotors()));
    // m_driverController.L2().onFalse(new RobotSetStateParallel(m_elevator, ElevatorState.def, m_arm, ArmState.in, m_intake, 0));

    m_driverController.R2().onTrue(new InstantCommand(() -> m_SwerveModule.driveSetPower(2)));
    m_driverController.L2().onTrue(new InstantCommand(() -> m_SwerveModule.steeringSetPosition(Rotation2d.fromDegrees(90))));
    m_driverController.R1().onTrue(new InstantCommand(() -> m_SwerveModule.disableMotors()));
    m_driverController.L1().onTrue(new InstantCommand(() -> m_SwerveModule.steeringSetPosition(Rotation2d.fromDegrees(0))));


    
    // m_driverController.cross().onTrue(new ResetPositionElevator(elevator, 0));
    SmartDashboard.putData("Reset Elevator Position", new ResetPositionElevator(m_elevator, 0).ignoringDisable(true));
    SmartDashboard.putData("Reset Arm Position", new ResetPositionArm(m_arm, 127).ignoringDisable(true));
  }

  public void print(){
    SmartDashboard.putNumber("Abs", m_SwerveModule.getAbsolutePosition().getDegrees());
    SmartDashboard.putNumber("steerAngle", m_SwerveModule.getSteeringAngle().getDegrees());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
