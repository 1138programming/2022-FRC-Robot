package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private TalonFX flywheelMotor;

  public Flywheel() {
    flywheelMotor = new TalonFX(KFlywheelMotor);
  }

  public void move(double speed) {
    flywheelMotor.set(ControlMode.PercentOutput, speed);
  }
}
