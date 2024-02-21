// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class A_HomeAllCommand extends Command {
  /** Creates a new A_HomeAllCommand. */
  public A_HomeAllCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shouldersubsystem, RobotContainer.wristsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.wristsubsystem.enablemotionmagic(Constants.k_WristHomePosition);
    new WaitCommand(0.25);
    RobotContainer.shouldersubsystem.enablemotionmagic(Constants.k_ShoulderHomePosition);
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
