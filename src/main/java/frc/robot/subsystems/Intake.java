package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;



public class Intake extends SubsystemBase {
  private TalonSRX swivelIntakeMotor;
  private Victor spinIntakeMotor;
  private DigitalInput bottomLimitSwitch;
  private DigitalInput topLimitSwitch;

  public Intake() {
    swivelIntakeMotor = new TalonSRX(KSwivelIntakeMotor);
    spinIntakeMotor = new Victor(KSpinIntakeMotor);
  }

  public void moveSwivel(double speed) {
    swivelIntakeMotor.set(ControlMode.PercentOutput, speed);
  }
  public void moveSpin(double speed) {
    spinIntakeMotor.set(speed);
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }
  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }
}
