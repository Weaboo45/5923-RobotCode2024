// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;


public class SimpleAutonomous extends Command {
  private ScoringSubsystem subsystem;
  private SwerveDrivetrain drivetrain;
  private int phase;
  private Timer timer = new Timer();

  /** Creates a new SimpleAutonomous. */
  public SimpleAutonomous(ScoringSubsystem subsystem, SwerveDrivetrain drivetrain) {
    addRequirements(subsystem, drivetrain);
    this.subsystem = subsystem;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    phase = 1;
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
    switch (phase){
       case 1: //drive out
       if(timer.get() < 5){
        drivetrain.swerveDrive( new Translation2d(1, 0), 0, true, false);
       } else {
        drivetrain.swerveDrive( new Translation2d( 0, 0), 0, true, false);
        phase++;
       }
       break;

      default:
        drivetrain.swerveDrive( new Translation2d( 0, 0), 0, true, false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
