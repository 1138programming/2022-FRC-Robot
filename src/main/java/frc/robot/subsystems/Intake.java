package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonSRX swivelIntakeMotor;
  private TalonSRX spinIntakeMotor;

  public Intake() {
    swivelIntakeMotor = new TalonSRX(KSwivelIntakeMotor);
    spinIntakeMotor = new TalonSRX(KSpinIntakeMotor);
  }

  public void moveSwivel(double swivelSpeed) {
    swivelIntakeMotor.set(ControlMode.PercentOutput, swivelSpeed);
  }

  public void moveSpin(double spinSpeed) {
    spinIntakeMotor.set(ControlMode.PercentOutput, spinSpeed);
  }
}
