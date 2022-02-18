package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;



public class Intake extends SubsystemBase {
  private TalonSRX swivelIntakeMotor;
  private VictorSPX spinIntakeMotor;
  private DigitalInput bottomLimitSwitch;
  private DigitalInput topLimitSwitch;
  private DutyCycleEncoder swivelMagEncoder;

  public Intake() {
    swivelIntakeMotor = new TalonSRX(KSwivelIntakeMotor);
    spinIntakeMotor = new VictorSPX(KSpinIntakeMotor);
    swivelMagEncoder = new DutyCycleEncoder(KSwivelIntakeEncoder);
  }

  public void moveSwivel(double speed) {
    swivelIntakeMotor.set(ControlMode.PercentOutput, speed);
  }
  public void moveSpin(double speed) {
    spinIntakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }
  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }
}
