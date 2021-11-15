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

  public Drivetrain() {

    mFrontLeft = new WPI_TalonSRX(Constants.kFrontLeft);
    mFrontRight = new WPI_TalonSRX(Constants.kFrontRight);
    mBackLeft = new WPI_TalonSRX(Constants.kBackLeft);
    mBackRight = new WPI_TalonSRX(Constants.kBackRight);

  }

  public void Drive(double pXPower, double pYPower, double pZPower){

    mFrontLeft.set(pXPower + pYPower + pZPower);
    mFrontRight.set(-pXPower + pYPower + pZPower);
    mBackLeft.set(pXPower - pYPower + pZPower);
    mBackRight.set(-pXPower - pYPower + pZPower);

  }
}
