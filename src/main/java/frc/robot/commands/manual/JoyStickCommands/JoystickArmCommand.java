package frc.robot.commands.manual.JoyStickCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class JoystickArmCommand extends Command{
    private ScoringSubsystem subsystem;
    //private Supplier<Double> armMovement;
    private Supplier<Boolean> intakeButtonForward, intakeButtonBackward, shooterButton, armUp, armDown;
    //private boolean forward, backward, shooterOn;

    public JoystickArmCommand(ScoringSubsystem subsystem, Supplier<Boolean> armUp, Supplier<Boolean> armDown,
    Supplier<Boolean> intakeButtonForward, Supplier<Boolean> intakeButtonBackward, Supplier<Boolean> shooterButton){
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.armUp = armUp;
        this.armDown = armDown;
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

    if(armUp.get()){
      subsystem.moveArm(1.0);
    }
    if(armDown.get()){
      subsystem.moveArm(-1.0);
    }
    if(armUp.get() == false && armDown.get() == false){
      subsystem.moveArm(0.0);
    }

    //double armMovementVal = armMovement.get();
    //subsystem.moveArm(armMovementVal);

    if(intakeButtonForward.get()){
        subsystem.intake(0.5);
    }
    if(intakeButtonBackward.get()){
        subsystem.intake(-0.5);
    }
    if(intakeButtonForward.get() == false && intakeButtonBackward.get() == false){
        subsystem.intake(0.0);
    }

    if(shooterButton.get()){
        subsystem.shooter(-0.5);
    } else {
      subsystem.shooter(0.0);
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
