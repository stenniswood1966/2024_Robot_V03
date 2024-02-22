// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class A_Steven extends Command {
  double elapsedTime = 0; //counts the number of 20ms cycles that has past
  /** Creates a new A_Steven. */
  public A_Steven() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.feedsubsystem, RobotContainer.intakesubsystem, RobotContainer.shootsubsystem, RobotContainer.shouldersubsystem, RobotContainer.wristsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elapsedTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elapsedTime >= 0 && elapsedTime <= 5) {
    RobotContainer.shouldersubsystem.enablemotionmagic(Constants.k_ShoulderShootPosition);
    }

    if (elapsedTime >= 25 && elapsedTime <= 30) {
    RobotContainer.wristsubsystem.enablemotionmagic(Constants.k_FiringSolutionAngle);
    }
  
    if (elapsedTime >= 75 && elapsedTime <= 100){
    RobotContainer.shootsubsystem.Shoot();
    if (Constants.k_shootmotor1speed >= Constants.k_FiringSolutionSpeed) {
    RobotContainer.feedsubsystem.Feed();
    }
    }

    if (elapsedTime >= 100) {
      RobotContainer.wristsubsystem.enablemotionmagic(Constants.k_WristHomePosition);
    }

    if (elapsedTime >= 125){
      RobotContainer.shouldersubsystem.enablemotionmagic(Constants.k_ShoulderHomePosition);
    }

    elapsedTime = elapsedTime + 1;
    System.out.println(elapsedTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shootsubsystem.Stop();
    RobotContainer.feedsubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elapsedTime >= 150) {
      return true;
    }
    else {
      return false;
    }
  }
}
