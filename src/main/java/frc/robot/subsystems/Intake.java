package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;



public class Intake extends SubsystemBase {
  private TalonSRX swivelIntakeMotor;
  private TalonSRX beltIntakeMotor;
  private TalonSRX hingeIntakeMotor;
  private DutyCycleEncoder swivelMagEncoder;
  private DutyCycleEncoder hingeMagEncoder;

  public Intake() {
    swivelIntakeMotor = new TalonSRX(KSwivelIntakeMotor);
    beltIntakeMotor = new TalonSRX(KBeltIntakeMotor);
    hingeIntakeMotor = new TalonSRX(KHingeIntakeMotor);
    swivelMagEncoder = new DutyCycleEncoder(KSwivelIntakeEncoder);
    hingeMagEncoder = new DutyCycleEncoder(KHingeIntakeEncoder);
  }

  public void moveSwivel(double swivelSpeed) {
    swivelIntakeMotor.set(ControlMode.PercentOutput, swivelSpeed);
  }
  public void moveBelt(double beltSpeed) {
    beltIntakeMotor.set(ControlMode.PercentOutput, beltSpeed);
  }
  public void movePivot(double pivotSpeed) {
    hingeIntakeMotor.set(ControlMode.Position, pivotSpeed);
  }
}
