package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ChangeTurretMode extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private String mode;
    /**
     * write the string as exactly "Passing", "Hub", or "Neutral"
     * @param _drivetrain
     * @param _mode
     */
    public ChangeTurretMode(CommandSwerveDrivetrain _drivetrain, String _mode){
        drivetrain =_drivetrain;
        mode=_mode;
        addRequirements(drivetrain);
    }
    public void initialize(){
        switch (mode) {
            case "Passing":
                drivetrain.passingTurretMode();
                break;
            case "Hub":
                drivetrain.hubTurretMode();
                break;
            case "Neutral":
                drivetrain.neutralTurretMode();
                
            default:
                break;
        }
    }
    public void execute(){}
    public void end(boolean interrupted){}
    public boolean isFinished(){return true;} 
}
