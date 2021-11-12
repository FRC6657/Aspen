// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {

  private final Intake mIntake;
  private double mSpeed;

  public RunIntake(Intake pIntake, double pSpeed) {
    
    mIntake = pIntake;
    mSpeed = pSpeed;

    addRequirements(mIntake);

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("running");
    mIntake.run(mSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
