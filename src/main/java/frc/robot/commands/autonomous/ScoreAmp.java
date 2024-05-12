// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ScoreAmp extends Command {
    private ScoringSubsystem subsystem;
  private int phase;
  private Timer timer = new Timer();

  /** Creates a new ScoreAmp. */
  public ScoreAmp(ScoringSubsystem subsystem) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
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
      case 1: //raise arm
       if(timer.get() < 1){
        subsystem.moveArm(0.25); //-.25
       } else {
        subsystem.moveArm(0);
        phase++;
       }
        break;

       case 2: // shoot note
       if(timer.get() < 4){
        subsystem.shooterOn();
        if(timer.get() < 3){
          subsystem.intake(1);
        }
       } else {
        subsystem.shooterOff();
        subsystem.intake(0);
        phase++;
       }
        break;
      default:
        subsystem.shooterOff();
        subsystem.intake(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
