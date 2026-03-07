package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class DeployIntake extends Command{
    private Intake intake;
    public DeployIntake(Intake _intake){
        intake = _intake;
        addRequirements(_intake);
    }
    public void initialize(){}
    public void execute(){
        if(intake.getIntakeAngle()>35){
        intake.setRetractorSpeed(Constants.Setpoints.retractorDeploySpeed);
        }else{intake.setRetractorSpeed(.1);}
    }
    public boolean isFinished(){return intake.getIntakeAngle()<5;}
    public void end(boolean interrupted){intake.stopIntakeRetractor();}
}
