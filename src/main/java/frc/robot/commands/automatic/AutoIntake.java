package frc.robot.commands.automatic;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ScoringSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutoIntake extends Command {
  private ScoringSubsystem subsystem;
  private Timer time = new Timer();

  /** Creates a new AutoBalance. */
  public AutoIntake(ScoringSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new InstantCommand(() -> subsystem.intake(1));
    
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
