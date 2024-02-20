// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class A_ShootCommand extends Command {
  /** Creates a new A_ShootCommand. */
  public A_ShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new ShoulderPositionCommand(Constants.k_ShoulderShootPosition)
    .withTimeout(0.25)
    .andThen(new WristPositionCommand(Constants.k_WristShootPosition))
    .withTimeout(0.5)
    .andThen(new ShootCommand())
    .withTimeout(Constants.ShootDelayTime)
    .andThen(new FeedCommand())
    .withTimeout(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
