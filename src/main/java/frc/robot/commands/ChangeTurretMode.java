package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretMode;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ChangeTurretMode extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private TurretMode mode;
    /**
     * write the string as exactly "Passing", "Hub", or "Neutral"
     * @param _drivetrain
     * @param _mode
     */
    public ChangeTurretMode(CommandSwerveDrivetrain _drivetrain, TurretMode _mode){
        drivetrain =_drivetrain;
        mode=_mode;
        addRequirements(drivetrain);
    }
    public void initialize(){
        switch (mode) {
            case PASSING:
                drivetrain.passingTurretMode();
                break;
            case HUB:
                drivetrain.hubTurretMode();
                break;
            case NEUTRAL:
                drivetrain.neutralTurretMode();
                break; 
            default:
                break;
        }
    }
    public void execute(){}
    public void end(boolean interrupted){}
    public boolean isFinished(){return true;} 
}
