package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends SubsystemBase {
  private TalonSRX swivelIntakeMotor;
  private VictorSPX spinIntakeMotor;
  private DigitalInput bottomLimitSwitch;
  private DigitalInput topLimitSwitch;

  public Intake() {
    swivelIntakeMotor = new TalonSRX(KSwivelIntakeMotorTalon);
    spinIntakeMotor = new VictorSPX(KSpinIntakeVictor);
    swivelIntakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    bottomLimitSwitch = new DigitalInput(10);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Epic", swivelIntakeMotor.getSelectedSensorPosition());
  }

  public void moveSwivel(double speed) {
    swivelIntakeMotor.set(ControlMode.PercentOutput, speed);
  }
  public void moveSpin(double speed) {
    spinIntakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public boolean getBottomLimitSwitch() {
    SmartDashboard.putBoolean("getBottomLimitSwitch", bottomLimitSwitch.get());
    return bottomLimitSwitch.get();
  }
  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }


}
