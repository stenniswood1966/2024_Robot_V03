// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShootSubsystem;

public class A_ShootCommand extends Command {
  /** Creates a new A_ShootCommand. */
  public A_ShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shouldersubsystem, RobotContainer.wristsubsystem, RobotContainer.intakesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ShootSubsystem().Shoot();

    //put a delay here or do something with isFinished
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shootsubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        return false;

  }
}
