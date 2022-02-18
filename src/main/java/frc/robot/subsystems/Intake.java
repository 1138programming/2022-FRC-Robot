package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

public class Intake extends SubsystemBase {
  private TalonSRX intakeMotor;
	private static Pixy2 pixy;
  
  public Intake() {
    pixy = Pixy2.createInstance(new SPILink());
    intakeMotor = new TalonSRX(KIntakeMotor);
  }

  public void move(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
}

  public int getPixyColorRed() {
    return pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 1);
  }

  public int getPixyColorBlue() {
    return pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
