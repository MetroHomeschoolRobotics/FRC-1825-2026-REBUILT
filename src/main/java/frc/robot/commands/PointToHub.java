package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PointToHub extends Command {
    CommandSwerveDrivetrain drivetrain;
    CommandXboxController joystick;
    Pose2d hubPose;
     private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentricFacingAngle point = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.1);
    
    public PointToHub(CommandSwerveDrivetrain _drivetrain, CommandXboxController _joystick){
        drivetrain = _drivetrain;
        addRequirements(drivetrain);
    }
    public void initialize(){
        point.HeadingController.setPID(11.13,0,0.169);
    }
    public void execute(){drivetrain.applyRequest(()->point.withVelocityX(-joystick.getLeftX()*MaxSpeed)
        .withVelocityY(-joystick.getLeftY())
        .withTargetDirection(new Rotation2d(Units.degreesToRadians(drivetrain.angleToHub()+90)))
        
        );
    }
    public boolean isFinished(){return false;}
    public void end(boolean interrupted){}
}
