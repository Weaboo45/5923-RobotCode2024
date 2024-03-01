package frc.robot.commands.automatic;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class AutoShoot extends Command{
  private ScoringSubsystem subsystem;
  private Timer time = new Timer();

    public AutoShoot(ScoringSubsystem subsystem) {
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
        if(time.get() <= 2.0){

          if (time.get() <= 2.0 && time.get() >= 1.0) {
            subsystem.intake(1);
          } else {
            subsystem.intake(0);
          }

          subsystem.shooter(.25);
        } else {
          subsystem.shooter(0);
        }

        time.reset();
        time.stop();
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