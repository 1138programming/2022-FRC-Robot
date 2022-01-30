package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonSRX intakeMotor;

  public Intake() {
    intakeMotor = new TalonSRX(KIntakeMotor);
  }

  public void move(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }
}
