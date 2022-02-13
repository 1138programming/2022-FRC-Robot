package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Storage extends SubsystemBase {
  private TalonSRX bottomStorageMotor;
  private TalonSRX topStorageMotor;
  private DigitalInput colorSensor1;
  private DigitalInput colorSensor2;

  //set the motor
  // this is constructor
  public Storage(){
    bottomStorageMotor = new TalonSRX(KLeftStorageTalon);
    topStorageMotor = new TalonSRX(KRightStorageTalon);
    colorSensor1 = new DigitalInput(KBallSensor);
    colorSensor2 = new DigitalInput(KBallSensor);
  }
//set the speed   
  public void move(double topSpeed, double bottomSpeed) {
    topStorageMotor.set(ControlMode.PercentOutput, topSpeed);
    bottomStorageMotor.set(ControlMode.PercentOutput, bottomSpeed);
  }

  public boolean getBallSensor1(){
    return colorSensor1.get();
  }
  public boolean getBallSensor2(){
    return colorSensor2.get();
  }
}