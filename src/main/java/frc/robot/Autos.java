package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TurretMode;
import frc.robot.commands.ChangeTurretMode;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RunFullIndexing;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetInterpolatedShooterRPM;
import frc.robot.commands.SetShooterRPM;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Autos {
    private AutoFactory factory;
    private Turret turret;
    private Hood hood;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;
    private CommandSwerveDrivetrain drivetrain;
    final private SwerveRequest idle = new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0));
        
    public Autos(AutoFactory _factory,Turret _turret, Hood _hood, Intake _intake,Indexer _indexer,Shooter _shooter,CommandSwerveDrivetrain _drivetrain){
        factory = _factory;
        turret = _turret;
        hood = _hood;
        intake = _intake;
        indexer =_indexer;
        shooter = _shooter;
        drivetrain = _drivetrain;
    }
    public AutoRoutine midAuto(){
        AutoRoutine routine = factory.newRoutine("midAuto");

        final AutoTrajectory startToShoot = routine.trajectory("midToShooting");
        final AutoTrajectory shootToDepot = routine.trajectory("midShootingToDepot");

        routine.active().onTrue(startToShoot.resetOdometry()
        .andThen(startToShoot.cmd())
        .andThen(new ParallelDeadlineGroup(shootToDepot.cmd(),new SequentialCommandGroup(new ParallelRaceGroup(new DeployIntake(intake),Commands.waitSeconds(0.5)),new RunIntake(intake))))
        .andThen(new SequentialCommandGroup(new ChangeTurretMode(drivetrain, TurretMode.HUB),new SetInterpolatedShooterRPM(drivetrain, shooter)))
        .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer,shooter),Commands.waitSeconds(5)))
            .andThen(new SetShooterRPM(shooter,0))
            // startToShoot.resetOdometry()
            // .andThen(new ChangeTurretMode(drivetrain, "Hub"))
            // .andThen(startToShoot.cmd())//.deadlineFor(new AutoSetInterpolatedShooterRPM(drivetrain,shooter))
            // .andThen(Commands.waitSeconds(0.3)).deadlineFor(new RunFullIndexing(indexer))
            // .andThen(Commands.waitSeconds(.5)).deadlineFor(new DeployIntake(intake))
            // .andThen(shootToDepot.cmd())//.raceWith(new AutoSetInterpolatedShooterRPM(drivetrain,shooter), new SequentialCommandGroup(Commands.waitSeconds(3.6),new RunIntake(intake)))
            // .andThen(Commands.waitSeconds(5)).deadlineFor(new RunFullIndexing(indexer))
            );


        return routine;
    }
    public AutoRoutine leftAuto(){
        AutoRoutine routine = factory.newRoutine("leftAuto");
        final AutoTrajectory startToMid = routine.trajectory("leftTrenchToMid");
        final AutoTrajectory midToShoot = routine.trajectory("leftMidToShoot");
        //final SwerveRequest idle = new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0));
        routine.active().onTrue(
                  startToMid.resetOdometry().andThen(startToMid.cmd())
                  .alongWith(new SequentialCommandGroup(Commands.waitSeconds(1.5),new ParallelRaceGroup(new DeployIntake(intake),Commands.waitSeconds(0.3))),new ParallelRaceGroup(Commands.waitSeconds(2.2),new RunIntake(intake)))
                  .andThen(midToShoot.cmd()).alongWith(new ChangeTurretMode(drivetrain,TurretMode.HUB))
                  .andThen(drivetrain.applyRequest(()->idle))
                  .andThen(new SequentialCommandGroup(new SetInterpolatedShooterRPM(drivetrain,shooter),Commands.waitSeconds(1.5)))
                  .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer, shooter),new RunIntake(intake), Commands.waitSeconds(4)))
                  .andThen(new SetShooterRPM(shooter, 0))
            // andThen(new ParallelDeadlineGroup(startToMid.cmd(),new SequentialCommandGroup(new ParallelRaceGroup(new DeployIntake(intake),Commands.waitSeconds(0.5)),new RunIntake(intake))))
            // .andThen(midToShoot.cmd())
            // .andThen(new SequentialCommandGroup(new ChangeTurretMode(drivetrain, TurretMode.HUB),new SetInterpolatedShooterRPM(drivetrain, shooter)))
            
            // .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer,shooter),Commands.waitSeconds(5)))
            // .andThen(new SetShooterRPM(shooter,0))

        );
        return routine;
    }
    public AutoRoutine rightAuto(){
        AutoRoutine routine = factory.newRoutine("rightAuto");
        final AutoTrajectory startToMid = routine.trajectory("rightTrenchToMid");
        final AutoTrajectory midToShoot = routine.trajectory("rightMidToAlliance");
        final AutoTrajectory stopDrivetrain = routine.trajectory("Constraints");
        //final var idle = new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0);
        routine.active().onTrue(
            
            startToMid.resetOdometry().
            andThen(new ParallelDeadlineGroup(startToMid.cmd(),new SequentialCommandGroup(new ParallelRaceGroup(new DeployIntake(intake),Commands.waitSeconds(0.5)),new RunIntake(intake))))
            .andThen(midToShoot.cmd())
           .andThen(new ParallelRaceGroup(drivetrain.applyRequest(()->idle),Commands.waitSeconds(0.02)))
            .andThen(new SequentialCommandGroup(new ChangeTurretMode(drivetrain, TurretMode.HUB),new SetInterpolatedShooterRPM(drivetrain, shooter)))
            .andThen(Commands.waitSeconds(1))
         
            .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer,shooter),Commands.waitSeconds(2.5))).andThen(new SetShooterRPM(shooter,0))
            .andThen(startToMid.cmd()).alongWith(new RunIntake(intake))
            .andThen(drivetrain.applyRequest(()->idle))

            //startToMid.resetOdometry().andThen(startToMid.cmd()).andThen(midToShoot.cmd()).andThen(drivetrain.applyRequest(()->idle))
            );
        return routine;
    }
    public AutoRoutine centerShoot8(){
        AutoRoutine routine = factory.newRoutine("centerShoot8");

        final AutoTrajectory startToShoot = routine.trajectory("midToShooting");
        final AutoTrajectory shootingToDepot = routine.trajectory("midShootingToDepot");
        //final var idle = new SwerveRequest.Idle();
        routine.active().onTrue(
            startToShoot.resetOdometry().andThen(startToShoot.cmd()).andThen(new ParallelRaceGroup(Commands.waitSeconds(0.5),drivetrain.applyRequest(()->idle)))
           .andThen(new ParallelRaceGroup(new DeployIntake(intake),Commands.waitSeconds(0.3)))
           .andThen(new ChangeTurretMode(drivetrain, TurretMode.HUB))
            .andThen(new ParallelRaceGroup(new SetInterpolatedShooterRPM(drivetrain,shooter),Commands.waitSeconds(2)))
            .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer,shooter),Commands.waitSeconds(5)))
            .andThen(new SetShooterRPM(shooter, 0))
            .andThen(new ParallelDeadlineGroup(shootingToDepot.cmd(), new SequentialCommandGroup(Commands.waitSeconds(2),new RunIntake(intake))))
            .andThen(drivetrain.applyRequest(()->idle))
            .andThen(new SetInterpolatedShooterRPM(drivetrain, shooter))
            .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer,shooter),Commands.waitSeconds(5)))
            .andThen(new SetShooterRPM(shooter, 0))
        
            );
        return routine;
    }
   public AutoRoutine centerShoot8StayDepot(){
        AutoRoutine routine = factory.newRoutine("centerShoot8");

        final AutoTrajectory startToShoot = routine.trajectory("midToShooting");
        final AutoTrajectory shootingToDepot = routine.trajectory("midShootingToDepotStay");
       // final var idle = new SwerveRequest.Idle();
        routine.active().onTrue(
            startToShoot.resetOdometry().andThen(startToShoot.cmd()).andThen(new ParallelRaceGroup(Commands.waitSeconds(0.5),drivetrain.applyRequest(()->idle)))
           .andThen(new ParallelRaceGroup(new DeployIntake(intake),Commands.waitSeconds(0.3)))
           .andThen(new ChangeTurretMode(drivetrain, TurretMode.HUB))
            .andThen(new ParallelRaceGroup(new SetInterpolatedShooterRPM(drivetrain,shooter),Commands.waitSeconds(2)))
            .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer,shooter),Commands.waitSeconds(5)))
            .andThen(new SetShooterRPM(shooter, 0))
            .andThen(new ParallelDeadlineGroup(shootingToDepot.cmd(), new SequentialCommandGroup(Commands.waitSeconds(2),new RunIntake(intake))))
            .andThen(new SetInterpolatedShooterRPM(drivetrain, shooter))
            .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer,shooter),Commands.waitSeconds(5)))
            .andThen(new SetShooterRPM(shooter, 0))
        
            );
        return routine;
    }
  public AutoRoutine stationaryShoot8(){
        AutoRoutine routine = factory.newRoutine("stationaryShoot8");

        final AutoTrajectory startToShoot = routine.trajectory("midToShooting");
       // final var idle = new SwerveRequest.Idle();
        routine.active().onTrue(
           (new ChangeTurretMode(drivetrain, TurretMode.HUB))
           .andThen(new ParallelRaceGroup(new DeployIntake(intake),Commands.waitSeconds(0.5)))
           .andThen(new ParallelRaceGroup(new SetShooterRPM(shooter, 3600),Commands.waitSeconds(2)))
           .andThen(Commands.waitSeconds(3))
           .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer,shooter),Commands.waitSeconds(5)))
            .andThen(new SetShooterRPM(shooter, 0))
           
        );
        return routine;
    }
}