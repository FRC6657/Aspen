// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriverControl extends CommandBase {
  
  private final Drivetrain mDrivetrain;
  private final DoubleSupplier mXInput;
  private final DoubleSupplier mYInput;
  private final DoubleSupplier mZInput;

  public DriverControl(Drivetrain pDrivetrain, DoubleSupplier pXInput, DoubleSupplier pYInput, DoubleSupplier pZInput) {

    mDrivetrain = pDrivetrain;
    mXInput = pXInput;
    mYInput = pYInput;
    mZInput = pZInput;

    addRequirements(mDrivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivetrain.drive(mXInput.getAsDouble(), mYInput.getAsDouble(), mZInput.getAsDouble(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.drive(0, 0, 0, false);
  }
}
