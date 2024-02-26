// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;


public class SimpleAutonomous extends Command {
  private ScoringSubsystem subsystem;
  private Timer timer = new Timer();

  /** Creates a new SimpleAutonomous. */
  public SimpleAutonomous(ScoringSubsystem subsystem) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timedAutoSequence();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  private void timedAutoSequence() {
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
