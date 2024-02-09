package frc.lib;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final Rotation2d angleOffset;
  public final boolean driveMotorInverted;
  public final boolean angleMotorInverted;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   * @param driveMotorInverted
   * @param angleMotorInverted
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, Boolean driveMotorInverted, Boolean angleMotorInverted) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = canCoderID;
    this.angleOffset = angleOffset;
    this.driveMotorInverted = driveMotorInverted;
    this.angleMotorInverted = angleMotorInverted;
  }
}
