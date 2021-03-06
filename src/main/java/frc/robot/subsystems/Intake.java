// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  
  private final WPI_VictorSPX mMotor;

  public Intake() {
    mMotor = new WPI_VictorSPX(5);
  }

  public void run(double pSpeed){
    mMotor.set(pSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
