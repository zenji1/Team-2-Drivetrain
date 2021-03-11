/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.SubSystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;


public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */

   private static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontMotor;
   private static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontMotor;
   private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackMotor;
   private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackMotor;

   private static XboxController driveController = Robot.m_robotContainer.driveController;

   public static int MOTOR_ENCODER_CONDES_PER_REV = 2048;
   public static final double DIAMETER_INCHES = 5.0;
   private static final double IN_TO_M = 0.0254;

   public static final double WHEEL_DIAMETER = DIAMETER_INCHES *IN_TO_M;
   public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

   private static final double GEAR_RATIO = 12.75;

   private static final double TICKS_PER_METER = (MOTOR_ENCODER_CONDES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
   private static final double METERS_PER_TICKS = 1/TICKS_PER_METER;

   private static final int timeoutsMs = 10;

   private final PigeonIMU imu = RobotMap.drive_imu;

   


  public DriveTrain() {
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);

    leftFrontMotor.setInverted(false);
    rightFrontMotor.setInverted(true);
    leftBackMotor.setInverted(false);
    rightBackMotor.setInverted(true);

    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    leftFrontMotor.configVelocityMeasurementWindow(10);
    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

    rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    rightFrontMotor.configVelocityMeasurementWindow(10);
    rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

    leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    leftBackMotor.configVelocityMeasurementWindow(10);
    leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

    rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    rightBackMotor.configVelocityMeasurementWindow(10);
    rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

    leftFrontMotor.configNominalOutputForward(0, timeoutsMs);
    leftFrontMotor.configNominalOutputForward(0, timeoutsMs);
    leftFrontMotor.configPeakOutputForward(-1, timeoutsMs);
    leftFrontMotor.configPeakOutputReverse(-1, timeoutsMs);

    leftBackMotor.configNominalOutputForward(0, timeoutsMs);
    leftBackMotor.configNominalOutputForward(0, timeoutsMs);
    leftBackMotor.configPeakOutputForward(-1, timeoutsMs);
    leftBackMotor.configPeakOutputReverse(-1, timeoutsMs);

    rightFrontMotor.configNominalOutputForward(0, timeoutsMs);
    rightFrontMotor.configNominalOutputForward(0, timeoutsMs);
    rightFrontMotor.configPeakOutputForward(-1, timeoutsMs);
    rightFrontMotor.configPeakOutputReverse(-1, timeoutsMs);

    rightFrontMotor.configNeutralDeadband(0.001, timeoutsMs);
    rightBackMotor.configNeutralDeadband(0.001, timeoutsMs);
    leftFrontMotor.configNeutralDeadband(0.001, timeoutsMs);
    leftBackMotor.configNeutralDeadband(0.001, timeoutsMs);

    leftFrontMotor.setSensorPhase(true);
    rightFrontMotor.setSensorPhase(false);
    leftBackMotor.setSensorPhase(true);
    rightBackMotor.setSensorPhase(false);

    leftFrontMotor.setInverted(false);
    rightFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);
    rightBackMotor.setInverted(false);
  }

  public double getYaw(){
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public double getPitch(){
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public double getRoll(){
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public void ZeroYaw(){
    imu.setYaw(0, timeoutsMs);
    imu.setFusedHeading(0, timeoutsMs);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setModePercentVoltage(){
    leftFrontMotor.set(ControlMode.PercentOutput, 0);
    rightFrontMotor.set(ControlMode.PercentOutput, 0);
    leftBackMotor.set(ControlMode.PercentOutput, 0);
    rightBackMotor.set(ControlMode.PercentOutput, 0);

    leftFrontMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());
    leftBackMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());
    
  }

  public static void drive(double throttle, double rotate){
    leftFrontMotor.set(throttle + rotate);
    rightFrontMotor.set(throttle - rotate);
    leftBackMotor.set(throttle + rotate);
    rightBackMotor.set(throttle - rotate);
  }

  public void stop(){
    drive(0,0);
  }

  public static double getLeftEncoderPosition(){
    return leftFrontMotor.getSelectedSensorPosition();
  }

  public static double getRightEncoderPosition(){
    return rightFrontMotor.getSelectedSensorPosition();
  }

  public double distanceTravelledinTicks(){
    return ((getLeftEncoderPosition() + getRightEncoderPosition())/2);
  }

  public double getLeftEncoderVelocityMetersPerSecond(){
    double leftVelocityMPS = (leftFrontMotor.getSelectedSensorVelocity() * 10);
    leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
    return(leftVelocityMPS);
  }

  public double getRightEncoderVelocityMetersPerSecond(){
    double rightVelocityMPS = (rightFrontMotor.getSelectedSensorVelocity() * 10);
    rightVelocityMPS = rightVelocityMPS * METERS_PER_TICKS;
    return(rightVelocityMPS);
  }

  
  public double leftDistanceTravelledInMeters(){
    double left_dist = getLeftEncoderPosition() * METERS_PER_TICKS;
    return left_dist;
  }

  public double rightDistanceTravelledInMeters(){
    double right_dist = getRightEncoderPosition() * METERS_PER_TICKS;
    return right_dist;
  }

  public double distanceTravelledInMeters(){
    double distanceTravelled = (leftDistanceTravelledInMeters() + rightDistanceTravelledInMeters()) / 2;
    return distanceTravelled;
  }

  public void resetEncoders(){
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }
}
