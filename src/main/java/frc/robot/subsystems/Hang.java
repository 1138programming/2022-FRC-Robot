// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

//ctre
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//rev
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//wpilib
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  
  private TalonFX leftArmMotor;
  private TalonFX rightArmMotor;
  private CANSparkMax levelHangMotor;

  private Servo rightClawServo;
  private Servo leftClawServo;
  private Servo ratchetServo;

  private RelativeEncoder levelEncoder;

  private DigitalInput hangLimitBottom;

  private TalonFXConfiguration tFxConfiguration;

  public Hang() {
    //this is necessary if we want to read absolute encoder value from the TalonFX integrated encoder
    tFxConfiguration = new TalonFXConfiguration();
    tFxConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    leftArmMotor.configAllSettings(tFxConfiguration);
    rightArmMotor.configAllSettings(tFxConfiguration);

    leftArmMotor = new TalonFX(KLeftHangFalcon);
    rightArmMotor = new TalonFX(KRightHangFalcon);
    levelHangMotor = new CANSparkMax(KLevelHangNeo, MotorType.kBrushless);

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
    
    //linear servo config
    leftClawServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    rightClawServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    ratchetServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    
    hangLimitBottom = new DigitalInput(KHangLimitBottom);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("hangLimitBottom", getHangLimitBottom());
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
  
  //Arm mvoement in this function is unrestricted, can crush the bot
  public void moveArmsUnrestricted(double speed) {
      leftArmMotor.set(ControlMode.PercentOutput, speed);
      rightArmMotor.set(ControlMode.PercentOutput, -speed);
  }
  
  //Hang mvoement in this function is unrestricted, can crush the bot
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
