// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IncrementHoodAngle extends Command {

  Hood hood;
  double increment;
  double setpoint;
  CommandXboxController xbox;

  /** Creates a new IncrementHoodAngle. */
  public IncrementHoodAngle(Hood _hood, CommandXboxController _xbox) {
    hood = _hood;
    xbox = _xbox;
    addRequirements(_hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    increment = xbox.getRightY()/50;
    if (Math.abs(increment)<0.002) {
      increment = 0;
    }
    setpoint = setpoint+increment;
    setpoint = MathUtil.clamp(setpoint, -1, 1);
    hood.setSetpoint(setpoint);
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
