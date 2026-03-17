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
        
        intake.setRetractorSpeed(Constants.Setpoints.retractorDeploySpeed);
    }
    public boolean isFinished(){return false;}
    public void end(boolean interrupted){intake.stopIntakeRetractor();}
}
