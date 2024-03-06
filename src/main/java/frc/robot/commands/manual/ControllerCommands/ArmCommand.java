package frc.robot.commands.manual.ControllerCommands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.subsystems.ScoringSubsystem;

public class ArmCommand extends Command{
    private ScoringSubsystem subsystem;
    private Supplier<Double> rightTrigger, leftTrigger;
    private Supplier<Boolean> intakeButtonForward, intakeButtonBackward, shooterButton;
    //private boolean forward, backward, shooterOn;

    public ArmCommand(ScoringSubsystem subsystem, Supplier<Double> rightTrigger,Supplier<Double> leftTrigger,
    Supplier<Boolean> intakeButtonForward, Supplier<Boolean> intakeButtonBackward, Supplier<Boolean> shooterButton){
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        this.intakeButtonForward = intakeButtonForward;
        this.intakeButtonBackward = intakeButtonBackward;
        this.shooterButton = shooterButton;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightTriggerVal = rightTrigger.get() *.25;
    double leftTriggerVal = leftTrigger.get() *.25;
    double sumVal = rightTriggerVal - leftTriggerVal;
    subsystem.moveArm(sumVal);

    if(intakeButtonForward.get()){
        subsystem.intake(1);
    }
    if(intakeButtonBackward.get()){
        subsystem.intake(-1);
    }
    if(intakeButtonForward.get() == false && intakeButtonBackward.get() == false){
        subsystem.intake(0);
    }

    if(shooterButton.get()){
        subsystem.shooter(-.5);
    } else {
      subsystem.shooter(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
