package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class Intake extends SubsystemBase {
  private TalonSRX swivelIntakeMotor;
  private VictorSPX spinIntakeMotor;
  private DigitalInput bottomLimitSwitch;
  private DigitalInput topLimitSwitch;
  private DutyCycleEncoder swivelMagEncoder;
  private PIDController swivelController;
  private double intakeControllerkP, intakeControllerkI, intakeControllerkD;
  private final Pixy2 pixy;
  
  public Intake() {
    swivelIntakeMotor = new TalonSRX(KSwivelIntakeTalon);
    spinIntakeMotor = new VictorSPX(KSpinIntakeVictor);
    swivelIntakeMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    swivelController = new PIDController(intakeControllerkP, intakeControllerkI, intakeControllerkD);
    topLimitSwitch = new DigitalInput(kIntakeTopLimit);
    bottomLimitSwitch = new DigitalInput(kIntakeBottomLimit);

    pixy = Pixy2.createInstance(Pixy2.LinkType.SPI);
    pixyInit();
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
    SmartDashboard.putBoolean("getBottomLimitSwitch", bottomLimitSwitch.get());
    return bottomLimitSwitch.get();
  }
  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }
  //Pixy2 functions

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

  public int pixyInit() {
    SmartDashboard.putNumber("init", pixy.init(1));
    return pixy.init(1);
  }

  public ArrayList<Block> getRedPixyCashe() {
    pixy.getCCC().getBlocks(true, Pixy2CCC.CCC_SIG1, 3);
    return pixy.getCCC().getBlockCache();
  } 
  public ArrayList<Block> getBluePixyCashe() {
    pixy.getCCC().getBlocks(true, Pixy2CCC.CCC_SIG2, 3);
    return pixy.getCCC().getBlockCache();
  } 

  public void setLamp(){
    pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
		pixy.setLED(255, 255, 255); // Sets the RGB LED to full white
  }
	public Pixy2 getPixy() {
		return pixy;
	}
  public int getPixyColorRed() {
    return pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 3);
  }
  public int getPixyColorBlue() {
    return pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 3);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}