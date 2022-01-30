package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class Shooter extends SubsystemBase {
  private TalonSRX leftShooterMotor;
  private TalonSRX rightShooterMotor;

  public Shooter() {
    leftShooterMotor = new TalonSRX(KLeftShooterMotor);
    rightShooterMotor = new TalonSRX(KRightShooterMotor);

  }

  public void move(double speed) {
    rightShooterMotor.set(ControlMode.PercentOutput, speed);
    leftShooterMotor.set(ControlMode.PercentOutput, speed);
  }
}
