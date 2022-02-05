// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  private CANSparkMax leftSwingMotor;
  private CANSparkMax rightSwingMotor;
  private CANSparkMax levelHangMotor;
  private Servo clawLinearServo;
  private Servo ratchetLinearServo;
  private RelativeEncoder leftArmEncoder;
  private RelativeEncoder rightArmEncoder;
  private RelativeEncoder levelEncoder;

  private DigitalInput bottomLiftLimitSwitch;
  private DigitalInput topLiftLimitSwitch;
  private DigitalInput armLimitSwitch;

  public Hang() {
    leftSwingMotor = new CANSparkMax(KSwingHangMotor, MotorType.kBrushless);
    rightSwingMotor = new CANSparkMax(KSwingHangMotor, MotorType.kBrushless);
    levelHangMotor = new CANSparkMax(KLevelHangMotor, MotorType.kBrushless);

    leftArmEncoder = leftSwingMotor.getEncoder();
    rightArmEncoder = rightSwingMotor.getEncoder();
    levelEncoder = levelHangMotor.getEncoder();

    clawLinearServo = new Servo(KClawLinearServo);
    ratchetLinearServo = new Servo(KRatchetLinearServo);

    bottomLiftLimitSwitch = new DigitalInput(KLiftBottomLimit);
    topLiftLimitSwitch = new DigitalInput(KLiftTopLimit);
    armLimitSwitch = new DigitalInput(KArmsLimit);
  }
  public void move(double swingMotorSpeed, double levelMotorSpeed){
    leftSwingMotor.set(swingMotorSpeed);
    rightSwingMotor.set(swingMotorSpeed);
    levelHangMotor.set(levelMotorSpeed);
  }

  public void moveArms(double swingMotorSpeed) {
    leftSwingMotor.set(swingMotorSpeed);
    rightSwingMotor.set(swingMotorSpeed);
  }
  public void moveLift(double levelMotorSpeed) {
    levelHangMotor.set(levelMotorSpeed);
  }
  public void moveServo(double clawServoPos, double ratchetServoPos) {
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
