// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriverControl;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  
  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Conveyor mConveyor = new Conveyor();

  private final XboxController mController = new XboxController(0);
  private final Joystick mJoystick = new Joystick(1);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    CommandScheduler.getInstance().setDefaultCommand(mDrivetrain, 
      new DriverControl(mDrivetrain,
        () -> cubicDeadband(mController.getRawAxis(XboxController.Axis.kLeftY.value) * 0.8,1,0.1),
        () -> -cubicDeadband(mController.getRawAxis(XboxController.Axis.kLeftX.value) * 0.8,1,0.1),
        () -> cubicDeadband(mController.getRawAxis(XboxController.Axis.kRightX.value) * 0.8,1,0.1)
      )
    );

    JoystickButton bottomLeft = new JoystickButton(mJoystick, 3);
    JoystickButton bottomRight = new JoystickButton(mJoystick, 4);
    JoystickButton topLeft = new JoystickButton(mJoystick, 5);
    JoystickButton topRight = new JoystickButton(mJoystick, 6);

    //Powercells move up
    topLeft.whenHeld(new RunIntake(mIntake, 1));
    topRight.whenHeld(new RunConveyor(mConveyor, 0.5));

    //Powercells move down
    bottomLeft.whenHeld(new RunIntake(mIntake, -1));
    bottomRight.whenHeld(new RunConveyor(mConveyor, -0.5));


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


  // "I hate every character of this implementation but it works, and its late so dont @me" - Andrew
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new DriverControl(mDrivetrain, () -> 0, () -> 0.8, () -> 0).withTimeout(1)
    );
  }
}
