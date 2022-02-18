// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  
  private TalonFX leftSwingMotor;
  private TalonFX rightSwingMotor;
  private TalonFX levelHangMotor;
  private Servo clawLinearServo;
  private Servo ratchetLinearServo;
  private RelativeEncoder leftArmEncoder;
  private RelativeEncoder rightArmEncoder;
  private RelativeEncoder levelEncoder;
  // private PIDController hangController;

  private DigitalInput bottomLiftLimitSwitch;
  private DigitalInput topLiftLimitSwitch;
  private DigitalInput armLimitSwitch;

  // private double ki, kp, kd;

  public Hang() {
    // hangController = new PIDController(kp, ki, kd);
    leftSwingMotor = new TalonFX(KSwingHangMotor);
    rightSwingMotor = new TalonFX(KSwingHangMotor);
    levelHangMotor = new TalonFX(KLevelHangMotor);

    leftSwingMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightSwingMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    levelHangMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    clawLinearServo = new Servo(KClawLinearServo);
    ratchetLinearServo = new Servo(KRatchetLinearServo);

    bottomLiftLimitSwitch = new DigitalInput(KLiftBottomLimit);
    topLiftLimitSwitch = new DigitalInput(KLiftTopLimit);
    armLimitSwitch = new DigitalInput(KArmsLimit);
  }
  public void move(double swingMotorSpeed, double levelMotorSpeed){
    leftSwingMotor.set(ControlMode.PercentOutput, swingMotorSpeed);
    rightSwingMotor.set(ControlMode.PercentOutput, swingMotorSpeed);
    levelHangMotor.set(ControlMode.PercentOutput, levelMotorSpeed);
  }
  public void moveToPosition(double armPosition, double levelPosition){
    leftSwingMotor.set(ControlMode.Position, armPosition);
    rightSwingMotor.set(ControlMode.Position, armPosition);
    levelHangMotor.set(ControlMode.Position, levelPosition);
  }
  // public void moveArms(double swingMotorSpeed) {
  //   leftSwingMotor.set(ControlMode.PercentOutput, swingMotorSpeed);
  //   rightSwingMotor.set(ControlMode.PercentOutput, swingMotorSpeed);
  // }
  // public void moveLift(double levelMotorSpeed) {
  //   levelHangMotor.set(ControlMode.PercentOutput, levelMotorSpeed);
  // }
  public void moveClaw(double clawServoPos, double ratchetServoPos) {
    clawLinearServo.set(clawServoPos);
    ratchetLinearServo.set(ratchetServoPos);
  }

  public boolean getBottomLiftLimitSwitch() {
    return bottomLiftLimitSwitch.get();
  }
  public boolean getTopLiftLimitSwitch() {
    return topLiftLimitSwitch.get();
  }
  public boolean getArmsLimitSwitch() {
    return armLimitSwitch.get();
  }

  public double getLeftArmEncoder() {
    return leftArmEncoder.getPosition();
  }
  public double getRightArmEncoder() {
    return rightArmEncoder.getPosition();
  }
  public double getLevelEncoder() {
    return levelEncoder.getPosition();
  }
  // public void setFlywheelGains(double kP, double kI, double kD){
  //   hangController.setPID(kp, ki, kd);
  // }

  public void resetArmEncders() {
    leftArmEncoder.setPosition(0);
    rightArmEncoder.setPosition(0);
  }
  public void resetLevelEncoder() {
    levelEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
