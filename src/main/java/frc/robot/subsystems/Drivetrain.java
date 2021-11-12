// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonSRX mFrontLeft;
  private final WPI_TalonSRX mFrontRight;
  private final WPI_TalonSRX mBackLeft;
  private final WPI_TalonSRX mBackRight;

  private final MecanumDrive mMecanumDrive;

  private final PigeonIMU mPigeonIMU;

  private ShuffleboardTab mDriverstation = Shuffleboard.getTab("Driver Station");
  private int mLoops = 0;

    private NetworkTableEntry mFrontLeftEncoder = mDriverstation
      .add("Front Left Encoder", 0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    private NetworkTableEntry mFrontRightEncoder = mDriverstation
      .add("Front Right Encoder", 0)
        .withPosition(1, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
        private NetworkTableEntry mBackLeftEncoder = mDriverstation
      .add("Back Left Encoder", 0)
        .withPosition(0, 1)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    private NetworkTableEntry mBackRightEncoder = mDriverstation
      .add("Back Right Encoder", 0)
        .withPosition(1, 1)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    private NetworkTableEntry mGyroData = mDriverstation
      .add("Gyro", 0)
        .withPosition(2, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    
  

  public Drivetrain() {

    mFrontLeft = new WPI_TalonSRX(Constants.kFrontLeft);
    mFrontRight = new WPI_TalonSRX(Constants.kFrontRight);
    mBackLeft = new WPI_TalonSRX(Constants.kBackLeft);
    mBackRight = new WPI_TalonSRX(Constants.kBackRight);

    mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    mBackLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    mBackRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    mFrontLeft.setSelectedSensorPosition(0);
    mBackLeft.setSelectedSensorPosition(0);
    mFrontRight.setSelectedSensorPosition(0);
    mBackRight.setSelectedSensorPosition(0);

    mPigeonIMU = new PigeonIMU(7);
    mPigeonIMU.setFusedHeading(0);

    mFrontLeft.setInverted(true);
    mBackLeft.setInverted(true);

    mMecanumDrive = new MecanumDrive(mFrontLeft, mBackLeft , mFrontRight, mBackRight);

  }

  public void Drive(double pXPower, double pYPower, double pZPower){

    mMecanumDrive.driveCartesian(
      pXPower, pYPower, pZPower, mPigeonIMU.getFusedHeading());

  }

  public double[] getPowers(){
    double[] mPowers = {mFrontLeft.get(), mFrontRight.get(), mBackLeft.get(), mBackRight.get()};
    return mPowers;
  }

  @Override
  public void periodic() {
    mLoops += 1;
    if(mLoops == 5){
      mLoops = 0;
      mFrontLeftEncoder.setNumber(mFrontLeft.getSelectedSensorPosition());
      mFrontRightEncoder.setNumber(mFrontRight.getSelectedSensorPosition());
      mBackLeftEncoder.setNumber(mBackLeft.getSelectedSensorPosition());
      mBackRightEncoder.setNumber(mBackRight.getSelectedSensorPosition());
      mGyroData.setNumber(mPigeonIMU.getFusedHeading());
    }
  }
}
