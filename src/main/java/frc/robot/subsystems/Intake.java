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



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

public class Intake extends SubsystemBase {
  private TalonSRX swivelIntakeMotor;
  private VictorSPX spinIntakeMotor;
  private DigitalInput bottomLimitSwitch;
  private DigitalInput topLimitSwitch;
  private DutyCycleEncoder swivelMagEncoder;

  private static Pixy2 pixy;

  public Intake() {
    swivelIntakeMotor = new TalonSRX(KSwivelIntakeMotor);
    spinIntakeMotor = new VictorSPX(KSpinIntakeMotor);
    swivelMagEncoder = new DutyCycleEncoder(KSwivelIntakeEncoder);
    pixy = Pixy2.createInstance(new SPILink());
  }
  //Talon
  public void moveSwivel(double speed) {
    swivelIntakeMotor.set(ControlMode.PercentOutput, speed);
  }
  //Encoder
  public void moveSpin(double speed) {
    spinIntakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }
  // Getters
  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }
  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }
  //Pixy2 functions
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
