// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NavigableMap;
import java.util.TreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class FiringSolutionSubsystem extends SubsystemBase {

  private NavigableMap<Double, Double> shooterSpeeds = new TreeMap<Double, Double>();
  private NavigableMap<Double, Double> wristAngle = new TreeMap<Double, Double>();

  /** Creates a new FiringSolutionSubsystem. */
  public FiringSolutionSubsystem() {
    setUpSpeedLookUpTable();
    setUpWristLookUpTable();
  }

  private void setUpSpeedLookUpTable() {
    shooterSpeeds.put(0.0, (double) 50);
    shooterSpeeds.put(30.0, (double) 50);
    shooterSpeeds.put(35.0, (double) 50);
    shooterSpeeds.put(40.0, (double) 50);
    shooterSpeeds.put(45.0, (double) 50);
    shooterSpeeds.put(50.0, (double) 50);
    shooterSpeeds.put(55.0, (double) 50);
    shooterSpeeds.put(60.0, (double) 50);
    shooterSpeeds.put(65.0, (double) 55);
    shooterSpeeds.put(70.0, (double) 55);
    shooterSpeeds.put(75.0, (double) 56);
    shooterSpeeds.put(80.0, (double) 60);
    shooterSpeeds.put(85.0, (double) 60);
    shooterSpeeds.put(90.0, (double) 60);
    shooterSpeeds.put(95.0, (double) 60);
    shooterSpeeds.put(100.0, (double) 60);
    shooterSpeeds.put(105.0, (double) 60);
    shooterSpeeds.put(110.0, (double) 60);
    shooterSpeeds.put(115.0, (double) 60);
    shooterSpeeds.put(999.0, (double) 60);
  }

    private void setUpWristLookUpTable() {
    wristAngle.put(0.0, (double) 0.319 + Constants.k_WristModifyPosition);
    wristAngle.put(30.0, (double) 0.319 + Constants.k_WristModifyPosition);
    wristAngle.put(35.0, (double) 0.319 + Constants.k_WristModifyPosition);
    wristAngle.put(40.0, (double) 0.319 + Constants.k_WristModifyPosition);
    wristAngle.put(45.0, (double) 0.330 + Constants.k_WristModifyPosition);
    wristAngle.put(50.0, (double) 0.337 + Constants.k_WristModifyPosition);
    wristAngle.put(55.0, (double) 0.344 + Constants.k_WristModifyPosition);
    wristAngle.put(60.0, (double) 0.356 + Constants.k_WristModifyPosition);
    wristAngle.put(65.0, (double) 0.362 + Constants.k_WristModifyPosition);
    wristAngle.put(70.0, (double) 0.365 + Constants.k_WristModifyPosition);
    wristAngle.put(75.0, (double) 0.372 + Constants.k_WristModifyPosition);
    wristAngle.put(80.0, (double) 0.375 + Constants.k_WristModifyPosition);
    wristAngle.put(85.0, (double) 0.379 + Constants.k_WristModifyPosition);
    wristAngle.put(90.0, (double) 0.386 + Constants.k_WristModifyPosition);
    wristAngle.put(95.0, (double) 0.388 + Constants.k_WristModifyPosition);
    wristAngle.put(100.0, (double) 0.390 + Constants.k_WristModifyPosition);
    wristAngle.put(105.00, (double) 0.392 + Constants.k_WristModifyPosition);
    wristAngle.put(110.00, (double) 0.392 + Constants.k_WristModifyPosition);
    wristAngle.put(115.00, (double) 0.392 + Constants.k_WristModifyPosition);
    wristAngle.put(999.0, (double) 0.392 + Constants.k_WristModifyPosition);
  }

  private double getDistance() {
    //figure out the distance to the target
    //math to calculate the distance goes here (d = (h2-h1) / tan(a1+a2)

    double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 23.5; 

    // distance from the target to the floor
    double goalHeightInches = 57.0; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    //Constants.k_LLDistanceToAprilTag = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }

  @Override
  public void periodic() {
    Constants.k_LLDistanceToAprilTag = getDistance();

    if (Constants.k_LLDistanceToAprilTag >= 36 && Constants.k_LLDistanceToAprilTag <= 115) {

    double s_closeDistance = shooterSpeeds.floorKey(Constants.k_LLDistanceToAprilTag);
    double s_farDistance = shooterSpeeds.ceilingKey(Constants.k_LLDistanceToAprilTag);
    double s_closeShooter = shooterSpeeds.floorEntry(Constants.k_LLDistanceToAprilTag).getValue();
    double s_farShooter = shooterSpeeds.ceilingEntry(Constants.k_LLDistanceToAprilTag).getValue();
    Constants.k_FiringSolutionSpeed = ((s_farShooter - s_closeShooter) / (s_farDistance - s_closeDistance))* (Constants.k_LLDistanceToAprilTag - s_farDistance) + s_farShooter;

    double closeDistance = wristAngle.floorKey(Constants.k_LLDistanceToAprilTag);
    double farDistance = wristAngle.ceilingKey(Constants.k_LLDistanceToAprilTag);
    double closeShooter = wristAngle.floorEntry(Constants.k_LLDistanceToAprilTag).getValue();
    double farShooter = wristAngle.ceilingEntry(Constants.k_LLDistanceToAprilTag).getValue();
    Constants.k_FiringSolutionAngle = ((farShooter - closeShooter) / (farDistance - closeDistance))* (Constants.k_LLDistanceToAprilTag - farDistance) + farShooter;
    }
    else {
      Constants.k_FiringSolutionSpeed = Constants.k_ShootDefaultSpeed;
      Constants.k_FiringSolutionAngle = Constants.k_WristShootPosition;
    }

  SmartDashboard.putNumber("FSS calculated distance: ", Constants.k_LLDistanceToAprilTag);
  SmartDashboard.putNumber("FSS speed: ", Constants.k_FiringSolutionSpeed);
  SmartDashboard.putNumber("FSS angle: ", Constants.k_FiringSolutionAngle);

  }
}
