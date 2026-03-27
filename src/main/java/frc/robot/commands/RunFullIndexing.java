package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class RunFullIndexing extends Command {
    private Indexer indexer;
    private Shooter shooter;
    private double forward = 0;
    public RunFullIndexing(Indexer _indexer,Shooter _shooter){
        indexer = _indexer;
        addRequirements(_indexer);
       shooter = _shooter;
    }
    public void initialize(){
         forward = 0;
    }
    public void execute(){
        //TO/DO check rotation directions later
        if(forward%5==0||forward%5==1){
            indexer.setBeltSpeed(-Constants.Setpoints.beltSpeed);
            forward ++;
        }else{
            indexer.setBeltSpeed(Constants.Setpoints.beltSpeed);
            forward ++;
        }
        
        indexer.setIndexerSpeed(Constants.Setpoints.indexerSpeed);
    }
    public void end(boolean interrupted){
        indexer.stopBelt();
        indexer.stopIndexer();
       // shooter.setRPM(0);
    }
    public boolean isFinished(){
        return false;
    }
}
