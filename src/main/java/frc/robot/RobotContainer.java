// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriverControl;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  
  private final Drivetrain mDrivetrain = new Drivetrain();

  private final XboxController mController = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    CommandScheduler.getInstance().setDefaultCommand(mDrivetrain, 
      new DriverControl(mDrivetrain,
        () -> -cubicDeadband(mController.getRawAxis(XboxController.Axis.kLeftY.value) * 0.5,1,0.1),
        () -> cubicDeadband(mController.getRawAxis(XboxController.Axis.kLeftX.value) * 0.5,1,0.1),
        () -> cubicDeadband(mController.getRawAxis(XboxController.Axis.kRightX.value) * 0.5,1,0.1)
      )
    );

    JoystickButton mA = new JoystickButton(mController, XboxController.Button.kA.value);
    JoystickButton mB = new JoystickButton(mController, XboxController.Button.kB.value);


  }

  private double cubicDeadband(double pInput, double pWeight, double pDeadband){

    double output;

    if(Math.abs(pInput) > pDeadband){
      output = (((pWeight * (Math.pow(pInput, 3)) + 1*(1 - pWeight) * pInput) - (Math.abs(pInput)) / pInput * (pWeight * (Math.pow(pDeadband, 3)) + (1 - pWeight) * pDeadband)) / (1 - (pWeight * (Math.pow(pDeadband, 3)) + (1 - pWeight) * pDeadband)));
    }
    else{
      output = 0;
    }
    return output;
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
