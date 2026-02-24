package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;

public class PassingModeActive extends Command {
      private Indexer indexer;
      private CommandSwerveDrivetrain drivetrain;
      private double allianceLine;

       private boolean poseHigher;
       private Pose2d pose;
    public PassingModeActive(Indexer _indexer,CommandSwerveDrivetrain _drivetrain){
        indexer = _indexer;
        drivetrain = _drivetrain;
        addRequirements(_indexer);
        addRequirements(_drivetrain);
    }
    public void initialize(){
       
        if(drivetrain.getAlliance() == Alliance.Red){
            allianceLine=Constants.FieldSetpoints.redAllianceZoneX;
            poseHigher = false;
        }else{
            allianceLine=Constants.FieldSetpoints.blueAllianceZoneX;
            poseHigher = true;
        }
    }
    public void execute(){

        pose = drivetrain.getRobotPose();
        if(poseHigher==true){
            if(pose.getX()>allianceLine&&
             !(pose.getY()>Constants.FieldSetpoints.lowerYValue&&
             pose.getY()<Constants.FieldSetpoints.upperYValue)){
                indexer.setBeltSpeed(-.3);
                indexer.setIndexerSpeed(-.3);
            }
        }else if (poseHigher==false){
            if(pose.getX()<allianceLine&&
             !(pose.getY()>Constants.FieldSetpoints.lowerYValue&&
             pose.getY()<Constants.FieldSetpoints.upperYValue)){
                indexer.setBeltSpeed(-.3);
                indexer.setIndexerSpeed(-.3);
            }
        }
            
    }
    public void end(){
        
    }
    public boolean isFinished(){
        return false;
    }
}
