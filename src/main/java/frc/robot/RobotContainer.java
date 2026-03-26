// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.logging.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AutoSetInterpolatedShooterRPM;
import frc.robot.commands.ChangeTurretMode;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.IncrementShooterRPM;
import frc.robot.commands.IncrementTurretAngle;
import frc.robot.commands.PointToHub;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunFullIndexing;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeBackwards;
import frc.robot.commands.SetHoodAngle;
import frc.robot.commands.SetInterpolatedShooterRPM;
import frc.robot.commands.SetInterpolatedShooterRPMSOTM;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.SetTurretAngle;
import frc.robot.commands.directdriveturrret;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentricFacingAngle point = new SwerveRequest.FieldCentricFacingAngle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController manipulatorXbox = new CommandXboxController(1);
    public final Turret turret = new Turret();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Shooter shooter = new Shooter();
    public final Hood hood = new Hood();

    public final IncrementTurretAngle incrementTurretAngle = new IncrementTurretAngle(turret, manipulatorXbox);
    //public final directdriveturrret incrementTurretAngle = new directdriveturrret(turret, manipulatorXbox);//IncrementTurretAngle(turret, manipulatorXbox);
    public final IncrementShooterRPM incrementShooterRPM = new IncrementShooterRPM(shooter, manipulatorXbox);
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(3);
    private final AutoChooser autoChooser = new AutoChooser();
    private final AutoFactory autoFactory;
    private double angleToHubContainer = 0;
    private  Autos autos;
       public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autos = new Autos(autoFactory, turret, hood, intake, indexer, shooter, drivetrain);
         createAutoChooser();
        configureBindings();
       
        DataLogManager.start();
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            //TODO check w/ Austin on this idea
            drivetrain.applyRequest(() ->
                drive.withVelocityX((Math.pow(-driverXbox.getLeftY(),3)) * MaxSpeed*(manipulatorXbox.leftBumper().getAsBoolean() ? .75:1)) // Drive forward with negative Y (forward)
                    .withVelocityY((Math.pow(-driverXbox.getLeftX(),3)) * MaxSpeed*(manipulatorXbox.leftBumper().getAsBoolean() ? .75:1)) // Drive left with negative X (left)
                    .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        driverXbox.x().whileTrue(new RunIntakeBackwards(intake));
        driverXbox.rightBumper().whileTrue((drivetrain.applyRequest(()->point.withVelocityX(-driverXbox.getLeftX())
        .withVelocityY(-driverXbox.getLeftY()).withHeadingPID(11.13,0,0.169)
        .withTargetDirection( new Rotation2d(drivetrain.angleToHub()-90)))));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverXbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverXbox.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))
        // ));
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverXbox.back().and(driverXbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverXbox.back().and(driverXbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverXbox.start().and(driverXbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverXbox.start().and(driverXbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        

        // Reset the field-centric heading on left bumper press.
        // driverXbox.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        // driverXbox.rightBumper().whileTrue(new SetShooterRPM(shooter, 3000));
        // drivetrain.registerTelemetry(logger::telemeterize);
        // driverXbox.y().whileTrue( drivetrain.driveToPose(new Pose2d(0,0,new Rotation2d(0)),2, 2,180,360));
        // driverXbox.x().onTrue(new SetTurretAngle(turret, 200));
        // driverXbox.y().whileTrue(new SetInterpolatedShooterRPM(drivetrain, shooter));
        // driverXbox.a().whileTrue(new SetTurretAngle(turret, 170));
        // manipulatorXbox.a().whileTrue(new RunIntake(intake));
        // manipulatorXbox.b().whileTrue(new SetShooterRPM(shooter, 0));
        // manipulatorXbox.leftStick().whileTrue(new SetTurretAngle(turret,manipulatorXbox.getLeftX()*180));
        // manipulatorXbox.y().whileTrue(new SetHoodAngle(hood, 50));
        
        driverXbox.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Actually start shooter, into the hub
        manipulatorXbox.y().whileTrue(new SetInterpolatedShooterRPM(drivetrain, shooter).andThen(new RunFullIndexing(indexer,shooter))); 
        // Starts the turret and hood tracking the hub       
        manipulatorXbox.a().whileTrue(new ChangeTurretMode(drivetrain, Constants.TurretMode.HUB).andThen(new SetHoodAngle(hood, Constants.Setpoints.defaultHoodAngle)).andThen(new SetInterpolatedShooterRPM(drivetrain,shooter))); 
        // Starts the turret and hood tracking the alliance wall
        manipulatorXbox.x().whileTrue(new ChangeTurretMode(drivetrain, Constants.TurretMode.PASSING).andThen(new SetHoodAngle(hood, Constants.Setpoints.passingHoodAngle)));
        manipulatorXbox.b().whileTrue(new ChangeTurretMode(drivetrain, Constants.TurretMode.NEUTRAL));
       
        manipulatorXbox.povRight().whileTrue(new ChangeTurretMode(drivetrain, Constants.TurretMode.HUBSOTM)
        .andThen(new SetInterpolatedShooterRPMSOTM(drivetrain,shooter))
        .alongWith(new SequentialCommandGroup(Commands.waitSeconds(0.7),new RunFullIndexing(indexer, shooter))));

        manipulatorXbox.rightBumper().whileTrue(new RunIntake(intake));
        
        //interpolation setter that never ends
        //Spins up the shooter to shooter into the hub, but doesn't run the indexer
        manipulatorXbox.leftBumper().whileTrue(new AutoSetInterpolatedShooterRPM(drivetrain, shooter));
        //Runs the indexer
        manipulatorXbox.leftTrigger().whileTrue(new RunFullIndexing(indexer,shooter));
        //Runs the shooter at a preset RPM, and runs the indexer
        manipulatorXbox.rightTrigger().whileTrue(new SetShooterRPM( shooter,1500).andThen(new RunFullIndexing(indexer,shooter))); 

        manipulatorXbox.povLeft().whileTrue(new SetShooterRPM(shooter, 0));
        manipulatorXbox.povDown().whileTrue(new DeployIntake(intake));
        manipulatorXbox.povUp().whileTrue(new RetractIntake(intake));

    CommandScheduler.getInstance().setDefaultCommand(shooter, incrementShooterRPM );
    CommandScheduler.getInstance().setDefaultCommand(turret, incrementTurretAngle);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
        return autoChooser.selectedCommand();
    }
    private void updateContainer(){
    angleToHubContainer = drivetrain.angleToHub();
    }
  
    private void createAutoChooser(){
        autoChooser.addRoutine("left", autos::leftAuto);
        autoChooser.addRoutine("right", autos::rightAuto);
        autoChooser.addRoutine("center", autos::midAuto);
        autoChooser.addRoutine("centerShoot8", autos::centerShoot8);
        autoChooser.addRoutine("stationaryShoot8", autos::stationaryShoot8);
        SmartDashboard.putData("auto chooser",autoChooser);
    }
    public void test(){
        SmartDashboard.putNumber("angleToHubContainer", angleToHubContainer);
        
        SmartDashboard.putNumber("interpolated rpm", shooter.getInterpolatedRPM(drivetrain.distanceToPose(Constants.FieldSetpoints.redHubPose)));
    }
    public void startUp(){
        hood.setPID(Constants.Setpoints.defaultHoodAngle);
        turret.setPID(0);//default angle 
        shooter.setRPM(0);
    }
    public void periodic() {
       
    }
}
