// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  
  private TalonFX leftArmMotor;
  private TalonFX rightArmMotor;
  private TalonSRX levelHangMotor;
  private Servo rightClawServo;
  private Servo leftClawServo;
  private Servo ratchetServo;

  private DigitalInput leftArmLimit;
  private DigitalInput rightArmLimit;
  private DigitalInput hangLimit;

  private final double kHangEncoderLimitPos = 6.9; //Change When Testing

  private final double kArmMaxForwardLimit = 6.9; //Change When Testing
  private final double kArmMaxReverseLimit = 0; //Encoder should reset everytime it hits the arm limit switches

  public Hang() {
    leftArmMotor = new TalonFX(KLeftHangMotor);
    rightArmMotor = new TalonFX(KRightHangMotor);
    levelHangMotor = new TalonSRX(KLevelHangMotor);

    leftArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    levelHangMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    leftArmMotor.setNeutralMode(NeutralMode.Brake);
    rightArmMotor.setNeutralMode(NeutralMode.Brake);
    
    leftClawServo = new Servo(KLeftClawServo);
    rightClawServo = new Servo(KRightClawServo);
    ratchetServo = new Servo(KRatchetServo);

    leftClawServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    rightClawServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    ratchetServo.setBounds(2.5, 1.8, 1.5, 1.2, 0.5);
    
    leftArmLimit = new DigitalInput(KLeftArmLimit);
    rightArmLimit = new DigitalInput(KRightArmLimit);
    hangLimit = new DigitalInput(KHangLimit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("hangLimit", getHangLimit());
    SmartDashboard.putBoolean("LeftArmLimit", getLeftArmLimit());
    SmartDashboard.putBoolean("rightArmLimit", getRightArmLimit());
    SmartDashboard.putNumber("leftArmEncoder", getLeftArmEncoder());
    SmartDashboard.putNumber("rightArmEncoder", getRightArmEncoder());
  }

  //Left Arm positive speed goes back, right arm positive speed goes forward
  public void moveArmsSpeed(double speed) {
    if (speed < 0){
      if(leftArmLimit.get()){
        leftArmMotor.set(ControlMode.PercentOutput, 0);
        resetLeftArmEncoder();
      }
      else {
        leftArmMotor.set(ControlMode.PercentOutput, speed);
      }
    }
    else {
      if (getLeftArmEncoder() <= kArmMaxForwardLimit) {
        leftArmMotor.set(ControlMode.PercentOutput, speed);
      }
      else {
        leftArmMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    if (speed < 0){
      if(rightArmLimit.get()){
        rightArmMotor.set(ControlMode.PercentOutput, 0);
        resetLeftArmEncoder();
      }
      else {
        rightArmMotor.set(ControlMode.PercentOutput, -speed);
      }
    }
    else {
      if (getRightArmEncoder() <= kArmMaxForwardLimit) {
        rightArmMotor.set(ControlMode.PercentOutput, -speed);
      }
      else {
        rightArmMotor.set(ControlMode.PercentOutput, 0);
      }
    }
  }
  
  public void moveArmsToPosition(double position) {
    if(leftArmLimit.get()){
      resetLeftArmEncoder();
    }
    if(rightArmLimit.get()){
      resetRightArmEncoder();
    }
    if (position > kArmMaxForwardLimit) {
      leftArmMotor.set(ControlMode.Position, kArmMaxForwardLimit);
      rightArmMotor.set(ControlMode.Position, kArmMaxForwardLimit);
    }
    else if (position < kArmMaxReverseLimit) {
      leftArmMotor.set(ControlMode.Position, kArmMaxReverseLimit);
      rightArmMotor.set(ControlMode.Position, kArmMaxReverseLimit);
    }
    else {
      leftArmMotor.set(ControlMode.Position, position);
      rightArmMotor.set(ControlMode.Position, position);
    }
  }

  //Negative speed is up
  public void moveLevelHangSpeed(double speed) {
    if (speed > 0 && getHangLimit()) {
      levelHangMotor.set(ControlMode.PercentOutput, 0);
      resetLevelHangEncoder();
    }
    else if (speed < 0 && (getLevelHangEncoder() > kHangEncoderLimitPos)) {
      levelHangMotor.set(ControlMode.PercentOutput, 0);
    }
    else {
      levelHangMotor.set(ControlMode.PercentOutput, -speed);
    }
  }
  
  public void moveClaw(double pos) {
    leftClawServo.set(pos);
    rightClawServo.set(pos);

  }
  public void moveRachet(double pos) {
    ratchetServo.set(pos);
  }
  
  //BAD!!! Arm mvoement in this function is unrestricted, can crush the bot
  public void moveArmsUnrestricted(double speed) {
      leftArmMotor.set(ControlMode.PercentOutput, speed);
      rightArmMotor.set(ControlMode.PercentOutput, -speed);
  }
  
  //BAD!!! Hang mvoement in this function is unrestricted, can crush the bot
  public void moveLeveHangUnrestricted(double speed) {
    levelHangMotor.set(ControlMode.PercentOutput, -speed);
  }
  

  public void moveLevelToPosition(double position) {
    levelHangMotor.set(ControlMode.Position, position);
  }

  public boolean getHangLimit() {
    return hangLimit.get();
  }
  public boolean getLeftArmLimit() {
    return leftArmLimit.get();
  }
  public boolean getRightArmLimit() {
    return rightArmLimit.get();
  }

  public double getLeftArmEncoder() {
    return leftArmMotor.getSelectedSensorPosition();
  }
  public double getRightArmEncoder() {
    return rightArmMotor.getSelectedSensorPosition();
  }
  public double getLevelHangEncoder() {
    return levelHangMotor.getSelectedSensorPosition();
  }

  public void resetLeftArmEncoder() {
    leftArmMotor.setSelectedSensorPosition(0);
  }
  public void resetRightArmEncoder() {
    rightArmMotor.setSelectedSensorPosition(0);
  }
  public void resetLevelHangEncoder() {
    levelHangMotor.setSelectedSensorPosition(0);
  }

}
