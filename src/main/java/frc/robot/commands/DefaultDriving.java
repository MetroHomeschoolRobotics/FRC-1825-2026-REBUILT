// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Predicate;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDriving extends Command {
  /** Creates a new DefaultDriving. */
  
 private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
 private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
 private CommandSwerveDrivetrain drivetrain;
 private CommandXboxController driverXbox;
 private Double FilteredLeftX;
 private Double FilteredLeftY;
 private Double FilteredRightX;
 private Double LimitedLeftX;
 private Double LimitedLeftY;
 private Double LimitedRightX;
 private Double AccelerationLimit;
 private Double SpeedLimit;
 private Double PreviousLeftX;
 private Double PreviousLeftY;
 private Double PreviousRightX;
 private CommandXboxController manipulatorXbox;

//  SlewRateLimiter filter = new SlewRateLimiter(5);
//  SlewRateLimiter filter2 = new SlewRateLimiter(5);
//  SlewRateLimiter filter3 = new SlewRateLimiter(5);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public DefaultDriving(CommandSwerveDrivetrain _drivetrain, CommandXboxController _driverXbox, CommandXboxController _manipulatorXbox) {
    drivetrain = _drivetrain;
    driverXbox = _driverXbox;
    manipulatorXbox = _manipulatorXbox;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PreviousLeftX = 0.0;
    PreviousLeftY = 0.0;
    PreviousRightX = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (drivetrain.hubTrackingSOTMEnabled) {
      AccelerationLimit = 2.0/50;
      SpeedLimit = 0.6;
    }
    else {
      AccelerationLimit = 3.5/50;
      SpeedLimit = 1.0;
    }

    LimitedLeftX = MathUtil.clamp(-driverXbox.getLeftX(), -SpeedLimit, SpeedLimit);
    LimitedLeftY = MathUtil.clamp(-driverXbox.getLeftY(), -SpeedLimit, SpeedLimit);
    LimitedRightX = -driverXbox.getRightX();
    //LimitedRightX = MathUtil.clamp(-driverXbox.getRightX(), -SpeedLimit, SpeedLimit);


    FilteredLeftX = MathUtil.clamp(Math.pow(LimitedLeftX,3), PreviousLeftX-AccelerationLimit, PreviousLeftX+AccelerationLimit);
    PreviousLeftX = FilteredLeftX;
    FilteredLeftY = MathUtil.clamp(Math.pow(LimitedLeftY,3), PreviousLeftY-AccelerationLimit, PreviousLeftY+AccelerationLimit);
    PreviousLeftY = FilteredLeftY;
    FilteredRightX = MathUtil.clamp(Math.pow(LimitedRightX,3), PreviousRightX-AccelerationLimit, PreviousRightX+AccelerationLimit);
    PreviousRightX = FilteredRightX;

    drivetrain.setControl(
      drive.withVelocityX((FilteredLeftY) * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY((FilteredLeftX) * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(FilteredRightX * MaxAngularRate) // Drive counterclockwise with negative X (left)
    );

    // drivetrain.applyRequest(() ->
    //   drive.withVelocityX((Math.pow(-driverXbox.getLeftY(),3)) * MaxSpeed*(manipulatorXbox.leftBumper().getAsBoolean() ? .75:1)) // Drive forward with negative Y (forward)
    //     .withVelocityY((Math.pow(-driverXbox.getLeftX(),3)) * MaxSpeed*(manipulatorXbox.leftBumper().getAsBoolean() ? .75:1)) // Drive left with negative X (left)
    //     .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    // );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
