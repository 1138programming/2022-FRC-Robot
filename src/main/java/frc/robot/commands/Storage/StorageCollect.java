package frc.robot.commands.Storage;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import static frc.robot.Constants.*;

public class StorageCollect extends CommandBase {
    private Storage storage;

    public StorageCollect(Storage storage){
        this.storage = storage;
        addRequirements(storage);
    }

    public void intialize(){}

    public void excute(){
        storage.getBallSensor1();
        storage.getBallSensor2();

        if(storage.getBallSensor1() == true)
        {
            storage.move(1,1)                               ;
            if(storage.getBallSensor1() == false)
                                                            {
                storage.move(0,1)                           ;
                                                            }
            if(storage.getBallSensor2() == true)
                                                            {
                storage.move(1,0)                           ;            
                if(storage.getBallSensor1() == true)        {
                    storage.move(0,0)                       ;
                                                            }
                                                            }
                                                            }
                                                            }

    public void end(boolean interrupted){}

    public boolean isFinished(){
        return false;
    }
}
