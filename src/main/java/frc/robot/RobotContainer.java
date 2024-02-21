// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MinSpeed = TunerConstants.kSpeedAt12VoltsMps / 2; // min speed used during go slow
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double POVSpeed = TunerConstants.kSpeedAt12VoltsMps / 6; //min speed used with POV buttons


  //subsystems used
  public static ShoulderSubsystem shouldersubsystem = new ShoulderSubsystem();
  public static LaserSubsystem lasersubsystem = new LaserSubsystem();
  public static FeedSubsystem feedsubsystem = new FeedSubsystem();
  public static IntakeSubsystem intakesubsystem = new IntakeSubsystem();
  public static ShootSubsystem shootsubsystem = new ShootSubsystem();
  public static WristSubsystem wristsubsystem = new WristSubsystem();
  public static FiringSolutionSubsystem firingsolutionsubsystem = new FiringSolutionSubsystem();
  public static ClimbSubsystem climbsubsystem = new ClimbSubsystem();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage); //10% deadband openloop driving
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentricFacingAngle fieldcentricfacingangle = new SwerveRequest.FieldCentricFacingAngle().withDeadband(MaxSpeed * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  

  //joystick for manual subsystem control
  public static CommandXboxController joystick2 = new CommandXboxController(2); //xbox360controller for manual control of mechanisms

  //XK-80 HID keypad
  private final XboxController m_operator1Controller = new XboxController(1);
  private JoystickButton Button_1 = new JoystickButton(m_operator1Controller, 1);
  //private JoystickButton Button_2 = new JoystickButton(m_operator1Controller, 2);
  //private JoystickButton Button_3 = new JoystickButton(m_operator1Controller, 3);
  private JoystickButton Button_4 = new JoystickButton(m_operator1Controller, 4);
  private JoystickButton Button_5 = new JoystickButton(m_operator1Controller, 5);
  private JoystickButton Button_6 = new JoystickButton(m_operator1Controller, 6);
  private JoystickButton Button_10 = new JoystickButton(m_operator1Controller, 10);
  private JoystickButton Button_20 = new JoystickButton(m_operator1Controller, 20);
  private JoystickButton Button_21 = new JoystickButton(m_operator1Controller, 21);
  private JoystickButton Button_22 = new JoystickButton(m_operator1Controller, 22);

  /* Path follower */
  //private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  //Auto Chooser
  private final SendableChooser<Command> autochooser;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    //assign driver joystick buttons to drivetrain functions
    //Robot centric driving "aka forwardStraight"
    /*
    joystick.y().toggleOnTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ).ignoringDisable(true));
    */

    //Go Slow mode
    joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MinSpeed) // Drive forward with negative Y (forward) / 2
                                        .withVelocityY(-joystick.getLeftX() * MinSpeed) // Drive left with negative X (left) / 2
                                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                                        ).ignoringDisable(true));

    //X-stop brake mode
    joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));

    //AutoAlign to apriltag
    //fieldcentricfacingangle.HeadingController can be found in the POV button section
    joystick.rightStick().whileTrue(drivetrain.applyRequest(() -> fieldcentricfacingangle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                        .withTargetDirection(Constants.k_steering_target) //this would be the angle to line up with
                                        ).ignoringDisable(true))
                                        .whileTrue(new AutoAlignCommand(drivetrain));

    //shoot button
    joystick.y().whileTrue(
      new ShootCommand()
      .alongWith(new FeedCommand()))
      .onFalse(new WristPositionCommand(Constants.k_WristHomePosition)
      .withTimeout(0.25)
      .andThen(new ShoulderPositionCommand(Constants.k_ShoulderHomePosition))
      );

    // reset the field-centric heading on left bumper press
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    //POV buttons slow mode auto rotate to zero
    fieldcentricfacingangle.HeadingController = new PhoenixPIDController(10.0, 0, 0);
    SendableRegistry.setName(fieldcentricfacingangle.HeadingController, "AutoAlign", "fcfa HeadingController");
    Rotation2d alignangle = Rotation2d.fromDegrees(0); //sets the angle the robot should face to zero
    joystick.pov(0).whileTrue(drivetrain.applyRequest(()->fieldcentricfacingangle.withVelocityX(POVSpeed).withVelocityY(0).withTargetDirection(alignangle)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(()->fieldcentricfacingangle.withVelocityX(-POVSpeed).withVelocityY(0).withTargetDirection(alignangle)));
    joystick.pov(90).whileTrue(drivetrain.applyRequest(()->fieldcentricfacingangle.withVelocityX(0.0).withVelocityY(-POVSpeed).withTargetDirection(alignangle)));
    joystick.pov(270).whileTrue(drivetrain.applyRequest(()->fieldcentricfacingangle.withVelocityX(0.0).withVelocityY(POVSpeed).withTargetDirection(alignangle)));



    //Xk-80 HID Port 1
    //Button_1.whileTrue(new IntakeCommand());
    // Button_2.whileTrue(new LoadCommand());

    // Will be a parallel race group that ends after one second with the two and three second commands getting interrupted.
    //button.onTrue(Commands.race(twoSecCommand, oneSecCommand, threeSecCommand));
    Button_1.onTrue(Commands.race(new IntakeCommand(), new LoadCommand()).withTimeout(5)); //commands run until the NoteisReady variable = true or timeout

    Button_5.onTrue(new PrepareToShootCommand());
    
    Button_6.onTrue(new PrepareToShootCommand());

    Button_10.whileTrue(new OutakeCommand());

    Button_20.whileTrue(new ShoulderManualCommand().alongWith(new WristManualCommand())); //stops MM from running
    

    Button_21.onTrue( //home
      new WristPositionCommand(Constants.k_WristHomePosition)
      .withTimeout(0.25)
      .andThen(new ShoulderPositionCommand(Constants.k_ShoulderHomePosition))
    );

    Button_22.whileTrue( //shoot
      new ShootCommand()
      .alongWith(new FeedCommand()))
      .onFalse(new WristPositionCommand(Constants.k_WristHomePosition)
      .withTimeout(0.25)
      .andThen(new ShoulderPositionCommand(Constants.k_ShoulderHomePosition))
      );
      
    //joystick2 used for testing manual commands
    joystick2.a().whileTrue(new WristManualCommand());
    joystick2.b().whileTrue(new PrepareToShootCommand());
    joystick2.y().whileTrue(new A_HomeAllCommand());


    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  private void namedcommands() {
  // Register Named Commands for pathplanner to use during autonomous
  NamedCommands.registerCommand("Prepare", new PrepareToShootCommand().withTimeout(5));
  NamedCommands.registerCommand("Intake", new A_IntakeLoadCommand().withTimeout(5));
  NamedCommands.registerCommand("Shoot", new A_ShootCommand().withTimeout(5));
  NamedCommands.registerCommand("Home", new A_HomeAllCommand().withTimeout(5));
}

  public RobotContainer() {
    configureBindings();
    namedcommands(); //pathplanner namedcommands

    //pathplanner sendablechooser
    autochooser = AutoBuilder.buildAutoChooser("None");
    SmartDashboard.putData("Auto Chooser", autochooser);

    SmartDashboard.putData(feedsubsystem);
    SmartDashboard.putData(intakesubsystem);
    SmartDashboard.putData(lasersubsystem);
    SmartDashboard.putData(shootsubsystem);
    SmartDashboard.putData(shouldersubsystem);
    SmartDashboard.putData(wristsubsystem);
  }



  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    //return runAuto;
    return autochooser.getSelected();
  }
}
