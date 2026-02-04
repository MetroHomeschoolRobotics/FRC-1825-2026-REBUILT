package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class StandbyMode extends Command {
    private double[] timerList = Constants.TimerConstants.shiftTimes;
    private double currentTime;
    private double nextShift;
    private boolean finished = false;
    public StandbyMode(){}
    public void initialize(){
        currentTime = Timer.getMatchTime();
        for (double d : timerList) {
            if(d<=currentTime){
                nextShift=d;
                break;
            }
        }
    }
    public void execute(){
        currentTime=Timer.getMatchTime();
        if(currentTime <=nextShift+0.5){
            finished=true;
        }
        
    }
    public void end(boolean interrupted){}
    public boolean isFinished(){
        return finished;
    }
}
