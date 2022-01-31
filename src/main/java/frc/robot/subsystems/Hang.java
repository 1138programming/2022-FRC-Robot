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
  private CANSparkMax swingHangMotor;
  private CANSparkMax levelHangMotor;
  private Servo clawLinearServo;
  private Servo ratchetLinearServo;
  

  public Hang() {
    swingHangMotor = new CANSparkMax(KSwingHangMotor, MotorType.kBrushless);
    levelHangMotor = new CANSparkMax(KLevelHangMotor, MotorType.kBrushless);

    clawLinearServo = new Servo(KClawLinearServo);
    ratchetLinearServo = new Servo(KRatchetLinearServo);
  }
  public void move(double swingMotorSpeed, double levelMotorSpeed){
    swingHangMotor.set(swingMotorSpeed);
    levelHangMotor.set(levelMotorSpeed);
  }

  public void moveServo(double clawServoPos, double ratchetServoPos) {
    clawLinearServo.set(clawServoPos);
    ratchetLinearServo.set(ratchetServoPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
