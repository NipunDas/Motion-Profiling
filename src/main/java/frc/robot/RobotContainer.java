// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.util.Units;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private static DriveTrain m_drive;

  private static Joystick leftJoy, rightJoy;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    leftJoy = new Joystick(Constants.leftJoyPort);
    rightJoy = new Joystick(Constants.rightJoyPort);

    m_drive = new DriveTrain();
    // Configure the button bindings
    configureButtonBindings();
  }

  public static Joystick getLeftJoy() {
    return leftJoy;
  }

  public static Joystick getRightJoy() {
    return rightJoy;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(),
    new Pose2d(1.0, 1.0, new Rotation2d(-45)), 
    new Pose2d(2.0, 2.0, new Rotation2d(0)), 
    new Pose2d(1.0, 3.0, new Rotation2d(45)),
    new Pose2d(-1.0, 5.0, new Rotation2d(45)), 
    new Pose2d(-2.0, 6.0, new Rotation2d(0)), 
    new Pose2d(-1.0, 7.0, new Rotation2d(-45)),
    new Pose2d(0.0, 8.0, new Rotation2d(0))), 
    config);

    RamseteCommand autoCommand = new RamseteCommand(trajectory, m_drive::getPosition, new RamseteController(2.0, 0.7), 
    m_drive.getFeedforward(), m_drive.getKinematics(), m_drive::getWheelSpeeds, m_drive.getLeftController(), 
    m_drive.getRightController(), m_drive::tankDrive, m_drive);
    
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
