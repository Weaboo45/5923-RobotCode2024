package frc.robot.commands.manual.JoyStickCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;


public class DriveJoystickSwerve extends Command {
  /*
   * Creates a new DriveMecanum.
   */

  private SwerveDrivetrain drivetrain;
  private Supplier<Double>  y, x, z, multiplier;
  private Supplier<Boolean> fieldTOrientated, zeroHeading;
  boolean fieldDrive = true, onOff = false;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.0);


  public DriveJoystickSwerve(SwerveDrivetrain drivetrain, Supplier<Double> yDirect, Supplier<Double> xDirect, 
  Supplier<Double> rotation, Supplier<Boolean> fieldTOrientated, Supplier<Boolean> zeroHeading, Supplier<Double> multiplier) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.zeroHeading = zeroHeading;
    this.y = yDirect;
    this.x = xDirect;
    this.z = rotation;
    this.multiplier = multiplier;
    this.fieldTOrientated = fieldTOrientated; // toggle
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(zeroHeading.get()){
      drivetrain.zeroHeading();
    }

    double mult = -multiplier.get() * 2 + 3; //trying to make -1 to 1 turn to 1 to 4
    if(mult > 4){
      mult = 4;
    }
    

    /* Get Values, Deadband */
    double translationVal = translationLimiter
        .calculate(MathUtil.applyDeadband(y.get(), Constants.SPEED_DEADBAND));
    double strafeVal = strafeLimiter
        .calculate(MathUtil.applyDeadband(x.get(), Constants.STRAFING_DEADBAND));
    double rotationVal = rotationLimiter
        .calculate(MathUtil.applyDeadband(-z.get(), Constants.ROTATION_DEADBAND));

    if(fieldTOrientated.get()){
      fieldDrive = !fieldDrive;
    }

    drivetrain.swerveDrive( new Translation2d(-translationVal * mult, strafeVal * mult),
      -rotationVal * 4, fieldDrive, false);
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