// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  
  private TalonFX leftArmMotor;
  private TalonFX rightArmMotor;
  private CANSparkMax levelHangMotor;
  private Servo rightClawServo;
  private Servo leftClawServo;
  private Servo ratchetServo;
  private RelativeEncoder levelEncoder;

  // private DigitalInput leftArmLimit;
  // private DigitalInput rightArmLimit;
  private DigitalInput hangLimitBottom;

  private final double kHangEncoderLimitPos = 6.9; //Change When Testing

  private final double kArmMaxForwardLimit = 6.9; //Change When Testing
  private final double kArmMaxReverseLimit = 0; //Encoder should reset everytime it hits the arm limit switches

  private TalonFXConfiguration tFxConfiguration;

  public Hang() {
    tFxConfiguration = new TalonFXConfiguration();
    tFxConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    

    leftArmMotor = new TalonFX(KLeftHangFalcon);
    rightArmMotor = new TalonFX(KRightHangFalcon);
    levelHangMotor = new CANSparkMax(KLevelHangNeo, MotorType.kBrushless);

    leftArmMotor.configAllSettings(tFxConfiguration);

    levelHangMotor.setInverted(true);

    leftArmMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightArmMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    levelEncoder = levelHangMotor.getEncoder();

    leftArmMotor.setNeutralMode(NeutralMode.Brake);
    rightArmMotor.setNeutralMode(NeutralMode.Brake);
    levelHangMotor.setIdleMode(IdleMode.kBrake);

    leftClawServo = new Servo(KLeftClawServo);
    rightClawServo = new Servo(KRightClawServo);
    ratchetServo = new Servo(KRatchetServo);

    leftClawServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    rightClawServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    ratchetServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    
    // leftArmLimit = new DigitalInput(KLeftArmLimit);
    // rightArmLimit = new DigitalInput(KRightArmLimit);
    hangLimitBottom = new DigitalInput(KHangLimitBottom);

    leftArmMotor.configAllSettings(tFxConfiguration);
    rightArmMotor.configAllSettings(tFxConfiguration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("hangLimitBottom", getHangLimitBottom());
  }

  //Left Arm positive speed goes back, right arm positive speed goes forward
  public void moveArmsSpeed(double speed) {
  //   if (speed < 0){
  //     if(leftArmLimit.get()){
  //       leftArmMotor.set(ControlMode.PercentOutput, 0);
  //       resetLeftArmEncoder();
  //     }
  //     else {
  //       leftArmMotor.set(ControlMode.PercentOutput, speed);
  //     }
  //   }
  //   else {
  //     if (getLeftArmEncoder() <= kArmMaxForwardLimit) {
  //       leftArmMotor.set(ControlMode.PercentOutput, speed);
  //     }
  //     else {
  //       leftArmMotor.set(ControlMode.PercentOutput, 0);
  //     }
  //   }

  //   if (speed < 0){
  //     if(rightArmLimit.get()){
  //       rightArmMotor.set(ControlMode.PercentOutput, 0);
  //       resetLeftArmEncoder();
  //     }
  //     else {
  //       rightArmMotor.set(ControlMode.PercentOutput, -speed);
  //     }
  //   }
  //   else {
  //     if (getRightArmEncoder() <= kArmMaxForwardLimit) {
  //       rightArmMotor.set(ControlMode.PercentOutput, -speed);
  //     }
  //     else {
  //       rightArmMotor.set(ControlMode.PercentOutput, 0);
  //     }
  //   }
  }
  
  public void moveArmsToPosition(double position) {
  //   if(leftArmLimit.get()){
  //     resetLeftArmEncoder();
  //   }
  //   if(rightArmLimit.get()){
  //     resetRightArmEncoder();
  //   }
  //   if (position > kArmMaxForwardLimit) {
  //     leftArmMotor.set(ControlMode.Position, kArmMaxForwardLimit);
  //     rightArmMotor.set(ControlMode.Position, kArmMaxForwardLimit);
  //   }
  //   else if (position < kArmMaxReverseLimit) {
  //     leftArmMotor.set(ControlMode.Position, kArmMaxReverseLimit);
  //     rightArmMotor.set(ControlMode.Position, kArmMaxReverseLimit);
  //   }
  //   else {
  //     leftArmMotor.set(ControlMode.Position, position);
  //     rightArmMotor.set(ControlMode.Position, position);
  //   }
  }

  //Negative raw speed is up
  public void moveLevelHangSpeed(double speed) {
    speed = -speed;
    if (!getHangLimitBottom()) {
      levelHangMotor.set(speed);
    }
    else if (getHangLimitBottom() && speed > 0)
    {
      levelHangMotor.set(0);
    }
    else {
      levelHangMotor.set(speed);
    }
  }
  
  public void moveClaw(double pos) {
    leftClawServo.set(pos);
    rightClawServo.set(pos);

  }
  
  //BAD!!! Arm mvoement in this function is unrestricted, can crush the bot
  public void moveArmsUnrestricted(double speed) {
      leftArmMotor.set(ControlMode.PercentOutput, speed);
      rightArmMotor.set(ControlMode.PercentOutput, -speed);
  }
  
  //BAD!!! Hang mvoement in this function is unrestricted, can crush the bot
  public void moveLeveHangUnrestricted(double speed) {
    // ratchetServo.set(kHangRatchetDistance);
    levelHangMotor.set(-speed);
  }
  
  public void moveHangRatchetServo(double pos) {
    ratchetServo.set(pos);
  }

  public void moveLevelToPosition(double position) {
    PIDController speedController = new PIDController(1, 0, 0);
    
    levelHangMotor.set(speedController.calculate(getLevelHangEncoder(), position));
  }

  public boolean getHangLimitBottom() {
    return !(hangLimitBottom.get());
  }
  // public boolean getLeftArmLimit() {
  //   return leftArmLimit.get();
  // }
  // public boolean getRightArmLimit() {
  //   return rightArmLimit.get();
  // }

  public double getLeftArmEncoder() {
    return leftArmMotor.getSelectedSensorPosition();
  }
  public double getRightArmEncoder() {
    return rightArmMotor.getSelectedSensorPosition();
  }
  public double getLevelHangEncoder() {
    return levelEncoder.getPosition();
  }

  public void resetLeftArmEncoder() {
    leftArmMotor.setSelectedSensorPosition(0);
  }
  public void resetRightArmEncoder() {
    rightArmMotor.setSelectedSensorPosition(0);
  }
  public void resetLevelHangEncoder() {
    levelHangMotor.getEncoder().setPosition(0);
  }

}
