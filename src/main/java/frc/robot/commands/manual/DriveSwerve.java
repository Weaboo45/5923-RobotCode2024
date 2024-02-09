/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manual;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;


public class DriveSwerve extends Command {
  /*
   * Creates a new DriveMecanum.
   */

  private SwerveDrivetrain drivetrain;
  private Supplier<Double>  y, x, z;
  private Supplier<Boolean> fieldTOrientated, togglespeed, resetGyro;
  boolean fieldDrive = true, onOff = false;
  double speed = 3.5;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.0);


  public DriveSwerve(SwerveDrivetrain drivetrain, Supplier<Double> yDirect, Supplier<Double> xDirect, 
  Supplier<Double> rotation, Supplier<Boolean> fieldTOrientated, Supplier<Boolean> togglespeed, Supplier<Boolean> resetGyro) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.togglespeed = togglespeed;
    this.y = yDirect;
    this.x = xDirect;
    this.z = rotation;
    this.resetGyro = resetGyro;
    this.fieldTOrientated = fieldTOrientated; // toggle
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(togglespeed.get()){
      onOff = !onOff;
    }
    if(onOff){
      speed = 4.0;
    }

    if(resetGyro.get()){
      drivetrain.zeroHeading();
    }

    /* Get Values, Deadband */
    double translationVal = translationLimiter
        .calculate(MathUtil.applyDeadband(y.get(), Constants.SPEED_DEADBAND));
    double strafeVal = strafeLimiter
        .calculate(MathUtil.applyDeadband(x.get(), Constants.STRAFING_DEADBAND));
    double rotationVal = rotationLimiter
        .calculate(MathUtil.applyDeadband(z.get(), Constants.ROTATION_DEADBAND));

    if(fieldTOrientated.get()){
      fieldDrive = !fieldDrive;
    }

    drivetrain.swerveDrive( new Translation2d(translationVal * speed, strafeVal * speed),
      rotationVal * 4, fieldDrive, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {;
    //drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}