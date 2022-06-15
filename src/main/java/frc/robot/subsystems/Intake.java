package frc.robot.subsystems;

import static frc.robot.Constants.*;

//rev
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//ctre
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//wpilib
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;


public class Intake extends SubsystemBase {
  private TalonSRX swivelIntakeMotor;
  private VictorSPX spinIntakeMotor;
  private DigitalInput bottomLimitSwitch;
  private DigitalInput topLimitSwitch;
  private PIDController swivelController;
  private double intakeControllerkP = 0.00028;
  private double intakeControllerkI = 0.000008;
  private double  intakeControllerkD = 0;
  
  public Intake() {
    swivelIntakeMotor = new TalonSRX(KSwivelIntakeTalon); //watch out for data port limit https://docs.ctre-phoenix.com/en/stable/ch13_MC.html#limit-switches
    spinIntakeMotor = new VictorSPX(KSpinIntakeVictor);
    //mag encoder plugged into motor controller data port
    swivelIntakeMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    //pid controller for swivel to pos
    swivelController = new PIDController(intakeControllerkP, intakeControllerkI, intakeControllerkD);

    topLimitSwitch = new DigitalInput(kIntakeTopLimit);
    bottomLimitSwitch = new DigitalInput(KIntakeBottomLimit);
  }

  public void moveSwivel(double speed) {
    double calcSpeed = speed;
    if (getTopLimitSwitch()) {
      if (speed > 0) {
        calcSpeed = 0;
        swivelIntakeMotor.setSelectedSensorPosition(0);
      }
      else {
        swivelIntakeMotor.setSelectedSensorPosition(0);
      }
    }
    else if (getBottomLimitSwitch()) {
      swivelIntakeMotor.setSelectedSensorPosition(kIntakeHuntPos);
      if (speed < 0) {
        calcSpeed = 0;
      }
    }
    swivelIntakeMotor.set(ControlMode.PercentOutput, calcSpeed);
  }
  
  public void swivelToPos(double setPoint) {  
    moveSwivel(-swivelController.calculate(getIntakeEncoderRaw(), setPoint));
  }

  public void moveSpin(double speed) {
    if (getTopLimitSwitch()) {
      spinIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
      SmartDashboard.putBoolean("Intake Spinning", false);
    }
    else {
      spinIntakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
      if (speed != 0) {
        SmartDashboard.putBoolean("Intake Spinning", true);
      }
      else {
        SmartDashboard.putBoolean("Intake Spinning", false);
      }
    }
  }
  
  public void resetEncoder(double position) {
    swivelIntakeMotor.setSelectedSensorPosition(position);
  }
  
  public void periodicResetEncoder(){
    if (getTopLimitSwitch()) {
      resetEncoder(0);
    }
    if (getBottomLimitSwitch()) {
      resetEncoder(kIntakeHuntPos);
    }
  }
  
  // Getters
  public boolean getBottomLimitSwitch() {
    return !bottomLimitSwitch.get();
  }
  public boolean getTopLimitSwitch() {
    return !(topLimitSwitch.get());
  }
  public double getIntakeEncoderRaw() {
    return swivelIntakeMotor.getSelectedSensorPosition();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake top limit", getTopLimitSwitch());
    SmartDashboard.putNumber("intake Encoder raw", getIntakeEncoderRaw());
    SmartDashboard.putBoolean("intake bottom limit", getBottomLimitSwitch());
    //constantly checks and see if encoder needs to be reset
    periodicResetEncoder();
  }
}