package frc.robot.commands.Storage;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import static frc.robot.Constants.*;

//Constructor
public class StorageCollect extends CommandBase {
    private Storage storage;

    public StorageCollect(Storage storage){
        this.storage = storage;
        addRequirements(storage);
    }

    public void intialize(){}

    public void execute(){
        boolean ballInTop = storage.getBallSensorTop();
        boolean ballInBottom = storage.getBallSensorBottom();
        //start by moving the bottom motor
        storage.move(0, 1);
        //logik loop
        //if the bottom sensor sees a ball, both motors will run
        if(ballInBottom)
        {
            //if the top sensor sees the ball (and the bottom sensor also sees ball), then both stop
            if (ballInTop) {
                storage.move(0, 0);
            }
            else {
                storage.move(1,1);
            }
        }
        
        //if the bottom sensor does NOT see a ball, only the bottom motor will run
        else 
        {
            storage.move(0, 1);
        }
    }

    public void end(boolean interrupted){}

    //stops return value
    public boolean isFinished(){
        return false;
    }
}
