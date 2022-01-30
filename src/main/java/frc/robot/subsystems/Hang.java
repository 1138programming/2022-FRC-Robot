// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  private CANSparkMax leftHangMotor;
  private CANSparkMax rightHangMotor;
  private CANSparkMax middleHangMotor;
  private Servo leftLinearServo;
  private Servo middleLinearServo;
  private Servo rightLinearServo;

  public Hang() {
    leftHangMotor = new CANSparkMax(KLeftHangMotor, MotorType.kBrushless);
    rightHangMotor = new CANSparkMax(KRightHangMotor, MotorType.kBrushless);
    middleHangMotor = new CANSparkMax(KMiddleHangMotor, MotorType.kBrushless);

    leftLinearServo = new Servo(KLeftLinearServo);
    middleLinearServo = new Servo(KMiddleLinearServo);
    rightLinearServo = new Servo(KRightLinearServo);
  }
  public void move(double leftMotorSpeed, double rightMotorSpeed, double middleMotorSpeed){
    leftHangMotor.set(leftMotorSpeed);
    rightHangMotor.set(rightMotorSpeed);
    middleHangMotor.set(middleMotorSpeed);
  }

  public void moveServo(double position) {
    leftLinearServo.set(position);
    middleLinearServo.set(position);
    rightLinearServo.set(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
