// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonSRX left = new WPI_TalonSRX(Constants.leftDrivePort);
  private WPI_TalonSRX right = new WPI_TalonSRX(Constants.rightDrivePort);

  private AHRS navx = new AHRS(SPI.Port.kMXP);
  Pose2d position;
  //Ultrasonic ultra = new Ultrasonic();

  //constants that need to be adjusted (rough estimates right now)
  double kWidth = 0.607;
  double kGearRatio = 1;
  double kWheelRadius = 0.076;
  double kTicksInRotation = 4096;
  double kS = 0.983;
  double kV = 3.34;
  double kA = 0.517;
  double kP = 2.14;
  public static double maxVelocity = 0.5, maxAcceleration = 0.5;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kWidth);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  PIDController leftController = new PIDController(kP, 0, 0);
  PIDController rightController = new PIDController(kP, 0, 0);

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    left.configFactoryDefault();
    left.setInverted(false);
    right.configFactoryDefault();
    right.setInverted(true);
    resetEncoders();
    navx.zeroYaw();
  }

  public void tankDrive(double leftPow, double rightPow) {
    left.set(ControlMode.PercentOutput, leftPow);
    right.set(ControlMode.PercentOutput, rightPow);
  }

  public void setVoltages(double leftPow, double rightPow) {
    left.set(ControlMode.PercentOutput, -1 * leftPow/12);
    right.set(ControlMode.PercentOutput, -1 * rightPow/12);
    System.out.println("Voltage: " + leftPow);
  }

  public void resetEncoders() {
    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
  }

  public double getLeftEncoderDistance() {
    return -1 * left.getSelectedSensorPosition(0) * 2 * Math.PI * kWheelRadius / (kTicksInRotation * kGearRatio);
  }

  public double getRightEncoderDistance() {
    return -1 * right.getSelectedSensorPosition(0) * 2 * Math.PI * kWheelRadius / (kTicksInRotation * kGearRatio);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle());
  }

  public Pose2d getPosition() {
    return position;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      -1 * left.getSensorCollection().getPulseWidthVelocity() * 20 * Math.PI * kWheelRadius / (kGearRatio * kTicksInRotation),
      -1 * right.getSensorCollection().getPulseWidthVelocity() * 20 * Math.PI * kWheelRadius / (kGearRatio * kTicksInRotation)
    );
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedForward;
  }

  public PIDController getLeftController() {
    return leftController;
  }

  public PIDController getRightController() {
    return rightController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public void periodic() {
    position = odometry.update(getHeading(), getLeftEncoderDistance(), getRightEncoderDistance());
    // This method will be called once per scheduler run
    tankDrive(RobotContainer.getLeftJoy().getY(), RobotContainer.getRightJoy().getY());
    //System.out.println(getWheelSpeeds().leftMetersPerSecond);
  }
}