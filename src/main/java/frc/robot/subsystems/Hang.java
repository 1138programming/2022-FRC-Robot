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

  private final double armEncoderFrontLimit = 6.9; //Change When Testing

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

  public void move(double swingMotorSpeed, double levelMotorSpeed){
    leftArmMotor.set(ControlMode.PercentOutput, swingMotorSpeed);
    rightArmMotor.set(ControlMode.PercentOutput, swingMotorSpeed);
    levelHangMotor.set(ControlMode.PercentOutput, levelMotorSpeed);
  }

  //Left Arm positive speed goes back, right arm positive speed goes forward
  public void moveArms(double speed) {
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
      if (getLeftArmEncoder() <= armEncoderFrontLimit) {
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
      if (getRightArmEncoder() <= armEncoderFrontLimit) {
        rightArmMotor.set(ControlMode.PercentOutput, -speed);
      }
      else {
        rightArmMotor.set(ControlMode.PercentOutput, 0);
      }
    }

  }

  public void moveArmsBad(double speed) {
    leftArmMotor.set(ControlMode.PercentOutput, speed);
    rightArmMotor.set(ControlMode.PercentOutput, -speed);

  }
  public void moveLeveHangBad(double speed) {
    levelHangMotor.set(ControlMode.PercentOutput, -speed);
  }

  //Negative speed is up
  public void moveLevel(double speed) {
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

  public void moveToPosition(double armPosition, double levelPosition){
    leftArmMotor.set(ControlMode.Position, armPosition);
    rightArmMotor.set(ControlMode.Position, armPosition);
    levelHangMotor.set(ControlMode.Position, levelPosition);
  }

  public void moveArmsToPosition(double position) {
    leftArmMotor.set(ControlMode.Position, position);
    rightArmMotor.set(ControlMode.Position, position);
  }

  public void moveLevelToPosition(double position) {
    levelHangMotor.set(ControlMode.PercentOutput, position);
    // levelHangMotor.set(ControlMode.Position, position);
  }
  // public void moveArms(double swingMotorSpeed) {
  //   leftSwingMotor.set(ControlMode.PercentOutput, swingMotorSpeed);
  //   rightSwingMotor.set(ControlMode.PercentOutput, swingMotorSpeed);
  // }
  // public void moveLift(double levelMotorSpeed) {
  //   levelHangMotor.set(ControlMode.PercentOutput, levelMotorSpeed);
  // }

  public void moveClaw(double pos) {
    leftClawServo.set(pos);
    rightClawServo.set(pos);
    SmartDashboard.putString("Claw", "ClawMoving");

  }
  public void moveRachet(double pos) {
    ratchetServo.set(pos);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("hangBottom", getHangLimit());
  }
}
