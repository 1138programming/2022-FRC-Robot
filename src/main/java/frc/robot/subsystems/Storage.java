package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase {
  //Defining motors
  private TalonSRX bottomStorageMotor;
  private TalonSRX topStorageMotor;
  //Defining encoders
  private DigitalInput ballSensorBottom;
  private DigitalInput ballSensorTop;

  //Constructor is constructing
  public Storage(){
    bottomStorageMotor = new TalonSRX(KLeftStorageTalon);
    topStorageMotor = new TalonSRX(KRightStorageTalon);
    ballSensorBottom = new DigitalInput(KBallSensorBottom);
    ballSensorTop = new DigitalInput(KBallSensorTop);
  }
  //move function, to make top run bottom not run, (1,0), vice versa
  public void move(double topSpeed, double bottomSpeed) {
    topStorageMotor.set(ControlMode.PercentOutput, topSpeed);
    bottomStorageMotor.set(ControlMode.PercentOutput, bottomSpeed);
  }

  public boolean getBallSensorTop(){
    return ballSensorTop.get();
  }
  public boolean getBallSensorBottom(){
    return ballSensorBottom.get();
  }
}