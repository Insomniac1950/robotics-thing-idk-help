// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
  private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;
  private static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontDriveMotor;
  private static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontDriveMotor;
  private static final double IN_TO_M= .0254;
  private static final int MOTOR_ENCODER_CODES_PER_REV = 2048;
  private static final double DIAMETER_INCHES = 5.0;
  private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double GEAR_RATIO = 12.75;
  private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
  private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

  public DriveSubsystem() {
    resetEncoders();
    leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());
    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    leftFrontMotor.configVelocityMeasurementWindow(16);
    leftFrontMotor.setStatusFramePeriod(StatusFrameEnchanced.Status_12_Feedback1, 5, 10);
    leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    leftBackMotor.configVelocityMeasurementWindow(16);
    leftBackMotor.setStatusFramePeriod(StatusFrameEnchanced.Status_12_Feedback1, 5, 10);
    rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    rightFrontMotor.configVelocityMeasurementWindow(16);
    rightFrontMotor.setStatusFramePeriod(StatusFrameEnchanced.Status_12_Feedback1, 5, 10);
    rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    rightBackMotor.configVelocityMeasurementWindow(16);
    rightBackMotor.setStatusFramePeriod(StatusFrameEnchanced.Status_12_Feedback1, 5, 10);
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);
    leftFrontMotor.setInverted(false);
    leftBackMotor.setInverted(false);
    rightFrontMotor.setInverted(false);
    rightBackMotor.setInverted(false);
    frontLeftMotor.set(ControlMode.Follower, backLeftMotor.getDeviceID());
    frontRightMotor.set(ControlMode.Follwoer, backRightMotor.getDeviceID());
    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);

    leftMotorFront.configNominalOutputForward(0,TIMEOUT_MS);
    leftMotorFront.configNominalOutputReverse(0,TIMEOUT_MS);
    leftMotorFront.configPeakOutputForward(1,TIMEOUT_MS);
    leftMotorFront.configPeakOutputReverse(-1,TIMEOUT_MS);

    leftMotorBack.configNominalOutputForward(0,TIMEOUT_MS);
    leftMotorBack.configNominalOutputReverse(0,TIMEOUT_MS);
    leftMotorBack.configPeakOutputForward(1,TIMEOUT_MS);
    leftMotorBack.configPeakOutputReverse(-1,TIMEOUT_MS);

    rightMotorFront.configNominalOutputForward(0,TIMEOUT_MS);
    rightMotorFront.configNominalOutputReverse(0,TIMEOUT_MS);
    rightMotorFront.configPeakOutputForward(1,TIMEOUT_MS);
    rightMotorFront.configPeakOutputReverse(-1,TIMEOUT_MS);

    rightMotorBack.configNominalOutputForward(0,TIMEOUT_MS);
    rightMotorBack.configNominalOutputReverse(0,TIMEOUT_MS);
    rightMotorBack.configPeakOutputForward(1,TIMEOUT_MS);
    rightMotorBack.configPeakOutputReverse(-1,TIMEOUT_MS);

    leftMotorFront.configNeutralDeadband(0.001, TIMEOUT_MS);
    leftMotorBack.configNeutralDeadband(0.001, TIMEOUT_MS);
    rightMotorFront.configNeutralDeadband(0.001, TIMEOUT_MS);
    rightMotorBack.configNeutralDeadband(0.001, TIMEOUT_MS);

    frontLeftMotor.setSensorPhase(true);
    frontRightMotor.setSensorPhase(false);
    backLeftMotor.setSensorPhase(true);
    backRightMotor.setSensorPhase(false);

    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);
    backLeftMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void resetEncoders() {
    backLeftMotor.setSelectedSensorPosition(0);
    backRightMotor.setSelectedSensorPosition(0);
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
  }
  public static void drive(double throttle, double rotate) {
    leftFrontMotor.set(throttle + rotate);
    rightFrontMotor.set(throttle - rotate);
    leftBackMotor.set(throttle + rotate);
    rightBackMotor.set(throttle - rotate);
  }
  public double getRightBackEncoderPosition() {
    return rightBackMotor.getSelectedSensorPosision();
  }
  public double distanceTravelledinTicks() {
    return (getLeftBackEncoderPosition() + getRightBackEncoderPosition()) / 2;
  }
  public double getLeftBackEncoderVelocityMetersPerSecond() {
    double leftVelocityMPS = (leftBackMotor.getSelectedSensorVelocity()*10);
    leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
    return (leftVelocityMPS);
  }
  public double leftDistanceTravelledInMeters() {
    double left_dist = getLeftBackEncoderPosition() * METERS_PER_TICKS;
    return left_dist;
  }
  public void stop() {
    drive(0,0);
  }
  public class JoystickDrive extends CommandBase {
    private final DriveSubSystem driveSubsystem;
    private final static XboxController driverController = RobotContainer.driverController;
  }
  public JoystickDrive(DriveSubsystem drivetrain) {
    driveSubsystem = drivetrain;
    addRequirements(driveSubsystem);
  }
  public void setModePercentVoltage() {
    leftFrontMotor.set(ControlMode.PercentOutput, 0);
    rightFrontMotor.set(ControlMode.PercentOutput, 0);
    leftBackMotor.set(ControlMode.PercentOutput, 0);
    rightBackMotor.set(ControlMode.PercentOutput, 0);
  }
}
