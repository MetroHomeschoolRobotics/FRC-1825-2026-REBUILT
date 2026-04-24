package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class FlickerIntakeUp extends Command{
    private Intake intake;
    private int timer;
    public FlickerIntakeUp(Intake _intake){
        intake = _intake;
    }
    public void initialize(){
        timer = 0;
    }
    public void execute(){
        if(timer%20<10){
            intake.setRetractorSpeed(Constants.Setpoints.retractorRetractSpeed+0.1);
        }else{
            intake.setRetractorSpeed(0);
        }
        timer++;
    }
    public void end(boolean interrupted){
        intake.setRetractorSpeed(0);
    }
}
