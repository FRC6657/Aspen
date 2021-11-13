// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriverControl extends CommandBase {
  
  private final Drivetrain mDrivetrain;
  private final DoubleSupplier mXInput;
  private final DoubleSupplier mYInput;
  private final DoubleSupplier mZInput;
  private BooleanSupplier mTurbo;

  public DriverControl(Drivetrain pDrivetrain, DoubleSupplier pXInput, DoubleSupplier pYInput, DoubleSupplier pZInput , BooleanSupplier pTurbo) {
    
    mDrivetrain = pDrivetrain;
    mXInput = pXInput;
    mYInput = pYInput;
    mZInput = pZInput;
    mTurbo = pTurbo;

    addRequirements(mDrivetrain);

  }

  @Override
  public void execute() {
    if(mTurbo.getAsBoolean()){
      mDrivetrain.Drive(mXInput.getAsDouble(), mYInput.getAsDouble(), mZInput.getAsDouble());
    }
    else{
      mDrivetrain.Drive(mXInput.getAsDouble()*0.5, mYInput.getAsDouble()*0.5, mZInput.getAsDouble()*0.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    mDrivetrain.Drive(0, 0, 0);
  }
}
