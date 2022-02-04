package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


public class DrivetrainSubsystem extends SubsystemBase {
  
  private WPI_TalonFX leftFalcon1, leftFalcon2, rightFalcon1, rightFalcon2;  
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  private PigeonIMU gyro;
  // private AHRS navX;

  // tracks robot pose (where it is on the field) using gyro & encoder values
  private DifferentialDriveOdometry odometry; 

  private DifferentialDriveWheelSpeeds wheelSpeeds; 
  private Pose2d pose; 
  private SimpleMotorFeedforward feedforward;

  private PIDController leftPIDController;
  private PIDController rightPIDController;

  // The robot's drive
  public final DifferentialDrive drive;

  private DriveMode driveMode;

  public enum DriveMode {
    TANK, CHEEZY, ARCADE;
  }

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {   
    leftFalcon1 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_1);
    leftFalcon2 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_2);
    rightFalcon1 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_1);
    rightFalcon2 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_2);

    leftMotors = new MotorControllerGroup(leftFalcon1, leftFalcon2);
    rightMotors = new MotorControllerGroup(rightFalcon1, rightFalcon2);
    rightMotors.setInverted(true);
    
    gyro = new PigeonIMU(DriveConstants.GYRO_ID);
    // navX = new AHRS(I2C.Port.kOnboard);

    odometry = new DifferentialDriveOdometry(getHeading());
    feedforward = new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeters, AutoConstants.kaVoltSecondsSquaredPerMeter);
    wheelSpeeds = new DifferentialDriveWheelSpeeds();

    // only need P
    leftPIDController = new PIDController(AutoConstants.kPDriveVel, 0.0, 0.0);
    rightPIDController = new PIDController(AutoConstants.kPDriveVel, 0.0, 0.0);

    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setDeadband(0.05);

    leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    leftFalcon1.setNeutralMode(NeutralMode.Brake);
    rightFalcon1.setNeutralMode(NeutralMode.Brake);
    resetEncoders();

    driveMode = DriveMode.CHEEZY;
  }

  @Override
  public void periodic(){
    wheelSpeeds.leftMetersPerSecond = getMetersPerSecondFromEncoder(leftFalcon1.getSelectedSensorVelocity()); 
    wheelSpeeds.rightMetersPerSecond = getMetersPerSecondFromEncoder(rightFalcon1.getSelectedSensorVelocity()); 
    // update pose using gyro and encoder values
    pose = odometry.update(getHeading(), wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);

    // System.out.println("navX: " + getTempAngle());
  }

  public void tankDrive(double leftpower, double rightpower) {
    drive.tankDrive(leftpower, rightpower);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    // might have to invert these if setInvert doesn't work
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public void cheezyDrive(double straight, double turn) {
    drive.curvatureDrive(straight, -turn, false);
  }

  public void arcadeDrive(double straight, double turn) {
    drive.arcadeDrive(straight, -turn);
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
    return (leftFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getRightEncoder() {
    return (-rightFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getAverageEncoder(){
    return ((getLeftEncoder()+getRightEncoder())/2);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double getAngle(){
    double [] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public void resetGyro(){
    gyro.setYaw(0);
  }

  public double getRawEncoder() {
    return leftFalcon1.getSelectedSensorPosition(); //temp method
  }

  public DriveMode getDriveMode(){
    return driveMode;
  }

  public void setDriveMode(DriveMode driveMode){
    this.driveMode = driveMode;
  }

  public DifferentialDriveWheelSpeeds getDifferentialDriveSpeeds(){
    return wheelSpeeds;
  }
  
  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  public PIDController getRightPIDController(){
    return rightPIDController;
  }

  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  /*
    takes raw falcon encoder value per 100ms and returns meters per second
    of the drivetrain wheels
    10.0 = 100 ms to 1s
    2.0 * PI * r = circumference of the wheel; rotations -> meters traveled
    2048.0 = CPR of falcon encoders
  */
  private double getMetersPerSecondFromEncoder(double raw){
    double ratio = (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO);
    return (10.0 * raw * 2.0 * Math.PI * Units.inchesToMeters(DriveConstants.WHEEL_RADIUS)) / (2048.0 * ratio);
  }

  private Rotation2d getHeading(){
    // negative applied because gyro (presumably) returns positive degrees 
    // as the gyro turns ccw; we want the opposite, as the opposite is true 
    // in math / on the unit circle
    return Rotation2d.fromDegrees(-getAngle()); 
  }

  // public double getTempAngle(){
  //   return navX.getAngle(); // ?
  // }

  // public void resetTempGyro(){
  //   navX.reset();
  // }
}
