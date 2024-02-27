// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Constants {

    public static Rotation2d k_steering_target = new Rotation2d(Math.toRadians(0)); //used by AutoAlignCommand to rotate to target
    public static final double k_MMRange = .005; //this is the range MM is considered finished

    public static boolean k_NoteisReady = false;
    public static final double ShootDelayTime = 0.5; //how long to wait before feeding note into shooter
    public static final double HomeDelayTime = 0.5; //how long to wait after shooting to home shoulder and wrist

    //Shoulder MM postions
    public static boolean k_ShoulderMMisMoving = false;
    public static final double k_ShoulderHomePosition = 2;
    public static final double k_ShoulderShootPosition = 11;
    public static final double k_ShoulderAmpPosition = 48.91;

    //Wrist MM Position
    public static boolean k_WristMMisMoving = false;
    public static double k_WristHomePosition = 0.0364; //0.051
    public static double k_WristAmpPosition = 0.41;

    //FiringSolutionSubsystem
    public static double k_LLDistanceToAprilTag = 0.0;
    public static double k_FiringSolutionSpeed = 45;
    public static double k_FiringSolutionAngle = 0.135;

    //shootersubsystem motor speeds
    public static double k_shootmotor1speed = 0.0;
    public static double k_shootmotor2speed = 0.0;
    

//Shooting postion
    //Preload
    public static double k_PreloadShooterSpeed = 45;
    public static double k_WristPreloadShootPosition = 0.137;

    //Position 1
    public static double k_Pos1ShooterSpeed = 60;
    public static double k_WristPos1ShootPosition = 0.190;

    //Postion 2
    public static double k_Pos2ShooterSpeed = 55;
    public static double k_WristPos2ShootPosition = 0.166;

    //Postion 3
    public static double k_Pos3ShooterSpeed = 60;
    public static double k_WristPos3ShootPosition = 0.190;
}
