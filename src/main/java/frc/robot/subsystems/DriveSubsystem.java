package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {
  
  private WPI_TalonFX leftFalcon1, leftFalcon2, rightFalcon1, rightFalcon2;  
  private Pose2d pose;
  private DifferentialDriveWheelSpeeds wheelSpeeds; 


  private final PigeonIMU m_gyro = new PigeonIMU(DriveConstants.GYRO_ID);
  
  private final DifferentialDriveOdometry m_odometry;

  // The robot's drive
  public final DifferentialDrive m_drive;

  private DriveMode driveMode;

  public enum DriveMode {
    TANK, CHEEZY, ARCADE;
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {   
    leftFalcon1 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_1);
    leftFalcon2 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_2);
    rightFalcon1 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_1);
    rightFalcon2 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_2);

    leftFalcon2.follow(leftFalcon1);
    leftFalcon2.setInverted(InvertType.FollowMaster);
    rightFalcon2.follow(rightFalcon1);
    rightFalcon2.setInverted(InvertType.FollowMaster);

    m_drive = new DifferentialDrive(leftFalcon1, rightFalcon1);
    m_drive.setDeadband(0.05);

    leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    leftFalcon1.setNeutralMode(NeutralMode.Brake);
    rightFalcon1.setNeutralMode(NeutralMode.Brake);
    resetEncoders();

    PigeonIMU m_gyro = new PigeonIMU(DriveConstants.GYRO_ID); 

    m_odometry = new DifferentialDriveOdometry(getHeading());

    pose = new Pose2d();
    wheelSpeeds = new DifferentialDriveWheelSpeeds();

  

    driveMode = DriveMode.CHEEZY;
  }

  @Override
  public void periodic(){
    wheelSpeeds.leftMetersPerSecond = getMetersPerSecondFromEncoder(leftFalcon1.getSelectedSensorVelocity()); 
    wheelSpeeds.rightMetersPerSecond = getMetersPerSecondFromEncoder(rightFalcon1.getSelectedSensorVelocity()); 
    // update pose using gyro and encoder values
    pose = m_odometry.update(getHeading(), wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    // System.out.println(getAngle());
    var translation = m_odometry.getPoseMeters().getTranslation();
    System.out.println("X: " + translation.getX() + ", Y: " + translation.getY());
  }

  public void tankDrive(double leftpower, double rightpower) {
    m_drive.tankDrive(leftpower, rightpower);
  }

  public void cheezyDrive(double straight, double turn) {
    m_drive.curvatureDrive(straight, -turn, false);
  }

  public void arcadeDrive(double straight, double turn) {
    m_drive.arcadeDrive(straight, -turn);
  }

  public void stopDrive() {
    leftFalcon1.set(ControlMode.PercentOutput, 0);
    rightFalcon1.set(ControlMode.PercentOutput, 0);
  }
    
  public void resetEncoders() {
    leftFalcon1.setSelectedSensorPosition(0);
    rightFalcon1.setSelectedSensorPosition(0);
  }

  public double getLeftEncoder() {
    return (leftFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE)  / (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getRightEncoder() {
    return (- rightFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getAverageEncoder(){
    return ((getLeftEncoder()+getRightEncoder())/2);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getAngle(){
    double [] ypr = new double[3];
    m_gyro.getYawPitchRoll(ypr);
    return ypr[0];
  }

  private Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-getAngle());
  } 

  public Pose2d getPose() {
    return pose;
  }

  public void resetGyro(){
    m_gyro.setYaw(0);
  }

  public double getRawEncoder() {
    return leftFalcon1.getSelectedSensorPosition(); //temp method
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return wheelSpeeds;
  }

  public DriveMode getDriveMode(){
    return driveMode;
  }

  public void setDriveMode(DriveMode driveMode){
    this.driveMode = driveMode;
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, getHeading());
  }

  private double getMetersPerSecondFromEncoder(double raw){
    double ratio = (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO);
    return (10.0 * raw * 2.0 * Math.PI * Units.inchesToMeters(DriveConstants.WHEEL_RADIUS)) / (2048.0 * ratio);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftFalcon1.setVoltage(leftVolts);
    leftFalcon2.setVoltage(leftVolts);
    rightFalcon1.setVoltage(rightVolts);
    rightFalcon2.setVoltage(rightVolts);
    m_drive.feed();
  }

}