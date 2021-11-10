// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonSRX mFrontLeft;
  private final WPI_TalonSRX mFrontRight;
  private final WPI_TalonSRX mBackLeft;
  private final WPI_TalonSRX mBackRight;

  private final PigeonIMU mPigeon;

  private final Translation2d mFrontLeftLocation = new Translation2d(-0.2775, 0.255);
  private final Translation2d mFrontRightLocation = new Translation2d(0.2775, 0.255);
  private final Translation2d mBackLeftLocation = new Translation2d(-0.2775, -0.255);
  private final Translation2d mBackRightLocation = new Translation2d(0.2775, -0.255);

  //Characterization should get these aswell
  private final PIDController mFrontLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController mFrontRightPIDController = new PIDController(1, 0, 0);
  private final PIDController mBackLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController mBackRightPIDController = new PIDController(1, 0, 0);

  private final MecanumDriveKinematics mKinematics = new MecanumDriveKinematics(
    mFrontLeftLocation, mFrontRightLocation, mBackLeftLocation, mBackRightLocation);

  //TODO: Run Characterization to get these values
  private final SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(1, 3);

  private SuppliedValueWidget<Double> mFrontLeftPower = Shuffleboard.getTab("Drivetrain")
    .addNumber("Front Left", () -> getPowers()[0]).withPosition(0,0).withSize(2, 1);
  private SuppliedValueWidget<Double> mFrontRightPower = Shuffleboard.getTab("Drivetrain")
    .addNumber("Front Right", () -> getPowers()[1]).withPosition(2,0).withSize(2, 1);
  private SuppliedValueWidget<Double> mBackLeftPower = Shuffleboard.getTab("Drivetrain")
    .addNumber("Back Left", () -> getPowers()[2]).withPosition(0,2).withSize(2, 1);
  private SuppliedValueWidget<Double> mBackRightPower = Shuffleboard.getTab("Drivetrain")
    .addNumber("Back Right", () -> getPowers()[3]).withPosition(2,2).withSize(2, 1);

  private SuppliedValueWidget<Double> mFrontLeftEncoder = Shuffleboard.getTab("Drivetrain")
    .addNumber("Front Left Encoder", () -> getEncoders()[0]).withPosition(0,1).withSize(2, 1);
  private SuppliedValueWidget<Double> mFrontRightEncoder = Shuffleboard.getTab("Drivetrain")
    .addNumber("Front Right Encoder", () -> getEncoders()[1]).withPosition(2,1).withSize(2, 1);
  private SuppliedValueWidget<Double> mBackLeftEncoder = Shuffleboard.getTab("Drivetrain")
    .addNumber("Back Left Encoder", () -> getEncoders()[2]).withPosition(0,3).withSize(2, 1);
  private SuppliedValueWidget<Double> mBackRightEncoder = Shuffleboard.getTab("Drivetrain")
    .addNumber("Back Right Encoder", () -> getEncoders()[3]).withPosition(2,3).withSize(2, 1);
  

  public Drivetrain() {

    mFrontLeft = new WPI_TalonSRX(Constants.kFrontLeftID);
    mFrontRight = new WPI_TalonSRX(Constants.kFrontRightID);
    mBackLeft = new WPI_TalonSRX(Constants.kBackLeftID);
    mBackRight = new WPI_TalonSRX(Constants.kBackRightID);

    mPigeon = new PigeonIMU(Constants.kPigeonID);

    mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    mBackLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    mBackRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

  }

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var mMecanumDriveWheelSpeeds =
        mKinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(mPigeon.getFusedHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    mMecanumDriveWheelSpeeds.normalize(3);
    setSpeeds(mMecanumDriveWheelSpeeds);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds pSpeeds){

    final double mFrontLeftFeedForward = mFeedForward.calculate(pSpeeds.frontLeftMetersPerSecond);
    final double mFrontRightFeedForward = mFeedForward.calculate(pSpeeds.frontRightMetersPerSecond);
    final double mBackLeftFeedForward = mFeedForward.calculate(pSpeeds.rearLeftMetersPerSecond);
    final double mBackRightFeedForward = mFeedForward.calculate(pSpeeds.rearRightMetersPerSecond);

    final double mFrontLeftOutput =
        mFrontLeftPIDController.calculate(
            mFrontLeft.getSelectedSensorVelocity() * Math.PI * 0.1524, pSpeeds.frontLeftMetersPerSecond);
    final double mFrontRightOutput =
        mFrontRightPIDController.calculate(
            mFrontRight.getSelectedSensorVelocity() * Math.PI * 0.1524, pSpeeds.frontRightMetersPerSecond);
    final double mBackLeftOutput =
        mBackLeftPIDController.calculate(
            mBackLeft.getSelectedSensorVelocity() * Math.PI * 0.1524, pSpeeds.rearLeftMetersPerSecond);
    final double mBackRightOutput =
        mBackRightPIDController.calculate(
            mBackRight.getSelectedSensorVelocity() * Math.PI * 0.1524, pSpeeds.rearRightMetersPerSecond);
   
    mFrontLeft.setVoltage(mFrontLeftOutput + mFrontLeftFeedForward);
    mFrontRight.setVoltage(mFrontRightOutput + mFrontRightFeedForward);
    mBackLeft.setVoltage(mBackLeftOutput + mBackLeftFeedForward);
    mBackRight.setVoltage(mBackRightOutput + mBackRightFeedForward);

  }

  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
      mFrontLeft.getSelectedSensorVelocity() * Math.PI * 0.1524,
      mFrontRight.getSelectedSensorVelocity() * Math.PI * 0.1524,
      mBackLeft.getSelectedSensorVelocity() * Math.PI * 0.1524,
      mBackRight.getSelectedSensorVelocity() * Math.PI * 0.1524
    );
  }

  public double[] getPowers(){
    double[] mPowers = {mFrontLeft.get(), mFrontRight.get(), mBackLeft.get(), mBackRight.get()};
    return mPowers;
  }

  public double[] getEncoders(){
  
    if(Robot.isReal()){
      double[] mReadings = {mFrontLeft.getSelectedSensorPosition(), mFrontRight.getSelectedSensorPosition(), mBackLeft.getSelectedSensorPosition(), mBackRight.getSelectedSensorPosition()};
      return mReadings;
    }
    else{
      double[] mReadings = {0,0,0,0};
      return mReadings;
    }

  }
}
