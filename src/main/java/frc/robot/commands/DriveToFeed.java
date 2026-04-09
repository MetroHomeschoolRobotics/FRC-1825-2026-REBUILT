package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToFeed extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;
    public DriveToFeed(CommandSwerveDrivetrain _drivetrain){
        drivetrain = _drivetrain;
    
    }
    public void initialize(){
        if(drivetrain.getAlliance()==Alliance.Red){
            if(drivetrain.distanceToPose(Constants.FieldSetpoints.redUpperFeedPose)>drivetrain.distanceToPose(Constants.FieldSetpoints.redLowerFeedPose)){
                targetPose = Constants.FieldSetpoints.redLowerFeedPose;
            }else{
                targetPose = Constants.FieldSetpoints.redUpperFeedPose;
            }

        }else{
            if(drivetrain.distanceToPose(Constants.FieldSetpoints.redUpperFeedPose)>drivetrain.distanceToPose(Constants.FieldSetpoints.redLowerFeedPose)){
                targetPose = Constants.FieldSetpoints.blueLowerFeedPose;
            }else{
                targetPose = Constants.FieldSetpoints.blueUpperFeedPose;
            }
        }
        drivetrain.driveToPose(targetPose, 3, 3, 180, 360).schedule();;
    }
    public void execute(){}
    public void end(boolean interrupted){}
    public boolean isFinished(){return false;}
}   
