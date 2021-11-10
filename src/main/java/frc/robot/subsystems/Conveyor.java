// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  
  private final WPI_TalonSRX mMotor;

  public Conveyor() {
    mMotor = new WPI_TalonSRX(6);
  }

  public void run(double pSpeed){
    mMotor.set(0);
  }
}
