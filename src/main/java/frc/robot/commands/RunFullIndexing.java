package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;

public class RunFullIndexing extends Command {
    private Indexer indexer;
    // private double forward = 0;
    public RunFullIndexing(Indexer _indexer){
        indexer = _indexer;
        addRequirements(_indexer);
    }
    public void initialize(){
        //  forward = 0;
    }
    public void execute(){
        // This was the code for oscilatting the belts.
        // if(forward%5==0||forward%5==1){
        //     indexer.setBeltSpeed(-Constants.Setpoints.beltSpeed);
        //     forward ++;
        // }else{
        //     indexer.setBeltSpeed(Constants.Setpoints.beltSpeed);
        //     forward ++;
        // }
        indexer.setBeltSpeed(Constants.Setpoints.beltSpeed);
        indexer.setIndexerSpeed(Constants.Setpoints.indexerSpeed);
    }
    public void end(boolean interrupted){
        indexer.stopBelt();
        indexer.stopIndexer();
    }
    public boolean isFinished(){
        return false;
    }
}
