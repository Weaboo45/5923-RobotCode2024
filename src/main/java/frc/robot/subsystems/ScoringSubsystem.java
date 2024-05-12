package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import org.littletonrobotics.junction.AutoLogOutput;

//import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
    //arm motors
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;

    //arm CANcoder
    private final RelativeEncoder motorEncoder;
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.kEncoderPort);

    //arm PID controller
    private final SparkPIDController armPID;

    //arm feed forward
    private final ArmFeedforward feedforward = 
    new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts, 
    ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    //shooter motors
    private CANSparkMax topShooterMotor;
    private CANSparkMax bottomShooterMotor;

    //intake motor
    private CANSparkMax intakeMotor;

    public ScoringSubsystem() {    
      //arm motors
      leftArmMotor = new CANSparkMax(Constants.leftArmMotorID, MotorType.kBrushless);
      rightArmMotor = new CANSparkMax(Constants.rightArmMotorID, MotorType.kBrushless);

      motorEncoder = rightArmMotor.getEncoder();
      motorEncoder.setPositionConversionFactor(ArmConstants.ARM_MOTOR_PCONVERSION);
        
      //shooter motors
      topShooterMotor = new CANSparkMax(Constants.topShooterMotorID, MotorType.kBrushless);
      bottomShooterMotor = new CANSparkMax(Constants.bottomShooterMotorID, MotorType.kBrushless);

      //intake motor
      intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushed);

      //motor configs
      configLeftArmMotor();
      configRightArmMotor();
      configBottorShooterMotor();
      configTopShooterMotor();
      configIntakeMotor();

      //reset encoder
      resetEncoder();
      absoluteEncoder.setPositionOffset(ArmConstants.angleOffsetRadian);

      armPID = rightArmMotor.getPIDController();
      armPID.setFeedbackDevice(motorEncoder);

      armPID.setP(ArmConstants.kP);
      armPID.setI(ArmConstants.kI);
      armPID.setD(ArmConstants.kD);
      
      SmartDashboard.putNumber("Arm Encoder angle", getRevEncoder());
      SmartDashboard.putNumber("Arm distance traveled", getDistance());

      SmartDashboard.putNumberArray("Arm Motor Temps", getTemp());
      SmartDashboard.putNumberArray("Arm Motor Currents", getCurrent());
  }

  public void hold(TrapezoidProfile.State setpoint){
    double ff = feedforward.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
    armPID.setReference(setpoint.position, ControlType.kPosition,0, ff);
  }

  public void runToPosition(TrapezoidProfile.State setpoint){
    if(getPos() >= ArmConstants.kUpperLimit){
      rightArmMotor.set(0);
      leftArmMotor.set(0);
    } else if(getPos() <= ArmConstants.kLowerLimit){
      rightArmMotor.set(0);
      leftArmMotor.set(0);
    } else{
      double ff = feedforward.calculate(setpoint.position*2*Math.PI, setpoint.position);
      armPID.setReference(setpoint.position, ControlType.kPosition,0, ff);
    }
  }

  public double getPos(){
    return absoluteEncoder.getAbsolutePosition();
  }

  public double[] getTemp(){
    double[] result = {rightArmMotor.getMotorTemperature(), leftArmMotor.getMotorTemperature()};
    return result;
  }

  public double[] getCurrent(){
    double[] result = {rightArmMotor.getOutputCurrent(), leftArmMotor.getOutputCurrent()};
    return result;
  }

  public boolean isInTolarance(double input, double target, double tolerance){
    double upLim = target + tolerance;
    double downLim = target - tolerance;

    if(downLim <= input && input <= upLim){
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    getRevEncoder();
    getDistance();
    Logger.recordOutput("Arm Distance", getRevEncoder());
  }

  private void configRightArmMotor() {
    rightArmMotor.restoreFactoryDefaults();
    rightArmMotor.setSmartCurrentLimit(15);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.enableVoltageCompensation(12);
    rightArmMotor.burnFlash();
  }

  private void configLeftArmMotor() {
    leftArmMotor.restoreFactoryDefaults();
    leftArmMotor.setSmartCurrentLimit(15);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.enableVoltageCompensation(12);
    leftArmMotor.burnFlash();
  }

  private void configTopShooterMotor() {
    topShooterMotor.restoreFactoryDefaults();
    topShooterMotor.setSmartCurrentLimit(30);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.enableVoltageCompensation(12);
    topShooterMotor.burnFlash();
  }

  private void configBottorShooterMotor() {
    bottomShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.setSmartCurrentLimit(30);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.enableVoltageCompensation(12);
    bottomShooterMotor.burnFlash();
  }

  private void configIntakeMotor() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(30);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.enableVoltageCompensation(12);
    intakeMotor.burnFlash();
  }

  public void resetEncoder(){
    absoluteEncoder.reset();
    double absolutePosition = getRevEncoder() - ArmConstants.angleOffsetRadian;
    motorEncoder.setPosition(absolutePosition);
  }

  @AutoLogOutput
  public double getRevEncoder(){
    return absoluteEncoder.getDistance() / (ArmConstants.ARM_DIAMETER / 2.0) * 100.0;
  }

  @AutoLogOutput
  public double getDistance(){
    return Math.abs(absoluteEncoder.getAbsolutePosition() - absoluteEncoder.getPositionOffset());
  }

  @AutoLogOutput
  public void moveArm(double pivotSpeed){
    rightArmMotor.set(pivotSpeed * -2.0);
    leftArmMotor.set(pivotSpeed * 2.0);
  }

  public void intake(double intakeSpeed){
    intakeMotor.set(intakeSpeed * 2);
  }

  public void intakeFoward(){
    intakeMotor.set(1.0);
  }

  public void intakeBackward(){
    intakeMotor.set(-1.0);
  }

  public void intakeOff(){
    intakeMotor.set(0.0);
  }

  public void shooterOn(){
    topShooterMotor.set(-.75);
    bottomShooterMotor.set(-.75);
  }

  public void shooterOff(){
    topShooterMotor.set(0.0);
    bottomShooterMotor.set(0.0);
  }

  public void shooter(double speed){
    topShooterMotor.set(speed);
    bottomShooterMotor.set(speed);
  }
}
