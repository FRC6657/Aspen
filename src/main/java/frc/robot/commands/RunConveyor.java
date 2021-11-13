// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class RunConveyor extends CommandBase {

  private final Conveyor mConveyor;
  private double mSpeed;

  public RunConveyor(Conveyor pConveyor, double pSpeed) {
    
    mConveyor = pConveyor;
    mSpeed = pSpeed;

    addRequirements(mConveyor);

  }

  @Override
  public void execute() {
    mConveyor.run(mSpeed);
  }


  @Override
  public void end(boolean interrupted) {
    mConveyor.run(0);
  }
}
