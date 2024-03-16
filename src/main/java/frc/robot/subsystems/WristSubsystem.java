// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class WristSubsystem extends SubsystemBase {
  static TalonFX motor1 = new TalonFX(6, TunerConstants.kCANbusName);
  static CANcoder cancoder = new CANcoder (4, TunerConstants.kCANbusName);

  MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    // Configure CANcoder to zero the magnet appropriately
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration(); // creates a default CANcoder configuration
    cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    //cc_cfg MagnetSensor.MagnetOffset = cancoder.getAbsolutePosition().getValueAsDouble(); // sets absoulte value as mag offset
    cc_cfg.MagnetSensor.MagnetOffset = 0.0; // Change this to match required offset
    cancoder.getConfigurator().apply(cc_cfg); // apply configuration to cancoder

    var fx_cfg = new TalonFXConfiguration(); //creates a default TalonFX configuration
    fx_cfg.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
    fx_cfg.Feedback.RotorToSensorRatio = 300;

    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake; // set brake after tuning
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.407; //use absolute position of cancoder in tuner x
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.041;

    /* Configure current limits */
    MotionMagicConfigs mm = new MotionMagicConfigs(); //creates a default motion magic congiguration
    mm.MotionMagicCruiseVelocity = 110; // RotorVelocity per second
    mm.MotionMagicAcceleration = 110; // Take approximately 0.5 seconds to reach max vel Take approximately 0.2 seconds to reach max accel
    mm.MotionMagicJerk = 0; //smooths out the transition from start/stop to cruise velocity
    fx_cfg.MotionMagic = mm;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 100; //output per unit of error in velocity (output/rps)
    slot0.kI = 0.0; //output per unit of integrated error in velocity (output/rotation)
    slot0.kD = 0.0; //output per unit of error derivative in velocity (output/(rps/s))
    slot0.kA = 0.2; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kV = 0.2; //output per unit of requested velocity (output/rps)
    slot0.kS = 0.2; //output to overcome static friction (output)
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    fx_cfg.Slot0 = slot0; //adds the slot0 config to the motors configuration file

    motor1.getConfigurator().apply(fx_cfg, 0.050); //apply configuration to motor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (cancoder.getVelocity().getValueAsDouble() == 0.0) {
      Constants.k_WristMMisMoving = false;
    }else {
      Constants.k_WristMMisMoving = true;
    }

    SmartDashboard.putBoolean("MM Status - Wrist: ", Constants.k_WristMMisMoving);
    SmartDashboard.putNumber("Wrist position: ", cancoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("k_WristModifyPosition: ", Constants.k_WristModifyPosition);
  }

  public void set(Double speed)  {
var motorRequest = new DutyCycleOut(speed); //Converts the double to DutyCycleOut
motor1.setControl(motorRequest); // Requests the motor to move
  }

  public void stop(){
    motor1.set(0);
  }
  public void enablemotionmagic(double targetpos) {
    // periodic, run Motion Magi with slot 0 configs,
    motor1.setControl(mmReq.withPosition(targetpos).withSlot(0));
  }

  public void disablemotionmagic() {
    motor1.set(0);
  }
  
  public static void addWristModifier() {
    Constants.k_WristModifyPosition = Constants.k_WristModifyPosition + 0.001;
  }

  public static void subtractWristModifier() {
    Constants.k_WristModifyPosition = Constants.k_WristModifyPosition - 0.001;
  }

  public static void resetWristModifier() {
    Constants.k_WristModifyPosition = 0.0;
  }
}
