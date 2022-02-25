package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
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
  private PIDController swivelController;
  private double intakeControllerkP, intakeControllerkI, intakeControllerkD;

  private static Pixy2 pixy;

  public Intake() {
    swivelIntakeMotor = new TalonSRX(KSwivelIntakeTalon);
    spinIntakeMotor = new VictorSPX(KSpinIntakeVictor);
    swivelIntakeMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    pixy = Pixy2.createInstance(new SPILink());
    swivelController = new PIDController(intakeControllerkP, intakeControllerkI, intakeControllerkD);
  }
  //Talon
  public void moveSwivel(double speed) {
    swivelIntakeMotor.set(ControlMode.PercentOutput, speed);
  }
  
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
    return pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 3);
  }

  public void resetEncoder() {
    // swivelMagEncoder.reset();
    swivelIntakeMotor.setSelectedSensorPosition(0);
  }
  public double getIntakeEncoderDeg() {
    // return (swivelMagEncoder.get() % 360);
    return (swivelIntakeMotor.getSelectedSensorPosition() % 360);
  }
  public void swivelToPos(double setPoint) {
    swivelIntakeMotor.set(TalonSRXControlMode.PercentOutput, swivelController.calculate(getIntakeEncoderDeg(), setPoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
