// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  private final SendableChooser<Command> mAutoChooser = new SendableChooser<>();
  private ShuffleboardTab mDriverstation = Shuffleboard.getTab("Driver Station");

  private final UsbCamera mIntakeCamera = CameraServer.getInstance().startAutomaticCapture();

  public RobotContainer() {
    configureButtonBindings();
  } 

  private class BaseLine extends SequentialCommandGroup {
    public BaseLine(){
      addCommands(
        new DriverControl(mDrivetrain, () -> 0.2, () -> 0, () -> 0, () -> true).withTimeout(3)
      );
    }
  }

  private class MaybeUnloadBalls extends SequentialCommandGroup {
    public MaybeUnloadBalls(){
      addCommands(
        new DriverControl(mDrivetrain, () -> 0.15, () -> 0, () -> 0.025, ()->true).withTimeout(4),
        //new DriverControl(mDrivetrain, () -> 0.5, () -> 0, () -> 0.2, ()->true).withTimeout(0.5),
        //new DriverControl(mDrivetrain, () -> 0.7, () -> 0, () -> 0.2, ()->true).withTimeout(0.5),
        new RunConveyor(mConveyor, 0.5).withTimeout(4)
      );
    }
  }

  private void configureButtonBindings() {

    mDriverstation.add("Camera",mIntakeCamera).withPosition(5, 0).withSize(3,4);

    CommandScheduler.getInstance().setDefaultCommand(mDrivetrain, 
      new DriverControl(mDrivetrain,
        () -> cubicDeadband(mController.getRawAxis(XboxController.Axis.kLeftY.value),0,0.1),
        () -> cubicDeadband(mController.getRawAxis(XboxController.Axis.kLeftX.value),0,0.1),
        () -> cubicDeadband(mController.getRawAxis(XboxController.Axis.kRightX.value),0,0.1),
        () -> mController.getRawButton(XboxController.Button.kBumperRight.value)
      )
    );

    JoystickButton bottomLeft = new JoystickButton(mJoystick, 3);
    JoystickButton bottomRight = new JoystickButton(mJoystick, 4);
    JoystickButton topLeft = new JoystickButton(mJoystick, 5);
    JoystickButton topRight = new JoystickButton(mJoystick, 6);

    //Powercells move up
    topLeft.whenHeld(new RunIntake(mIntake, 0.6));
    topRight.whenHeld(new RunConveyor(mConveyor, 0.4));

    //Powercells move down
    bottomLeft.whenHeld(new RunIntake(mIntake, -0.6));
    bottomRight.whenHeld(new RunConveyor(mConveyor, -0.4));

    mAutoChooser.setDefaultOption("Base Line", new BaseLine());
    mAutoChooser.addOption("MaybeScore", new MaybeUnloadBalls());

    mDriverstation.add("Autonomous Select", mAutoChooser).withPosition(0, 0).withSize(2, 1);   

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
    return mAutoChooser.getSelected();
  }
}
