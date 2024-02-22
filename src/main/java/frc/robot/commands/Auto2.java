// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Auto2 extends Command {
  double elapsedTime = 0; //counts the number of 20ms cycles that have occured
  /** Creates a new A_Steven. */
  public Auto2() {
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
    RobotContainer.shootsubsystem.Shoot();
    System.out.println("shoulder to position and shooter motors spinning up");
    }

    if (elapsedTime >= 25 && elapsedTime <= 30) {
    RobotContainer.wristsubsystem.enablemotionmagic(Constants.k_FiringSolutionAngle);
    System.out.println("wrist to position");
    }
  
    if (!Constants.k_WristMMisMoving && !Constants.k_ShoulderMMisMoving){ //shoot when wrist and shoulder stop moving
      System.out.println("shoulder and wrist stopped moving");
      if (Constants.k_shootmotor1speed >= Constants.k_FiringSolutionSpeed) { //wait until shooter is up to speed
      RobotContainer.feedsubsystem.Feed();
      System.out.println("feeding active");
      }
    }

    if (!Constants.k_NoteisReady) { //no note in shooter start to home wrist and shoulder
      RobotContainer.wristsubsystem.enablemotionmagic(Constants.k_WristHomePosition);
      RobotContainer.shouldersubsystem.enablemotionmagic(Constants.k_ShoulderHomePosition);
      System.out.println("home wrist and shoulder");
    }

    elapsedTime = elapsedTime + 1;
    System.out.println(elapsedTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shootsubsystem.Stop();
    RobotContainer.feedsubsystem.Stop();
    System.out.println("Auto2 stopped");
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
