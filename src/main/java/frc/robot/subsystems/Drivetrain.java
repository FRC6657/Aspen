// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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

    mFrontLeft = new WPI_TalonSRX(Constants.kFrontLeft);
    mFrontRight = new WPI_TalonSRX(Constants.kFrontRight);
    mBackLeft = new WPI_TalonSRX(Constants.kBackLeft);
    mBackRight = new WPI_TalonSRX(Constants.kBackRight);

    mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    mBackLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    mBackRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

  }

  public void Drive(double pXPower, double pYPower, double pZPower){

    mFrontLeft.set(pXPower + pYPower + pZPower);
    mFrontRight.set(-pXPower + pYPower + pZPower);
    mBackLeft.set(pXPower - pYPower + pZPower);
    mBackRight.set(-pXPower - pYPower + pZPower);

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
