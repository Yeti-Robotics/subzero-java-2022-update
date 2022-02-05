// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AllInCommand;
import frc.robot.commands.AllOutCommand;
import frc.robot.commands.drivetrain.ToggleShiftingCommand;
import frc.robot.commands.hood.TestHoodCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.ToggleIntakePistonCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.commands.turret.TurnToTargetPIDCommand;
import frc.robot.commands.turret.TurretTestCommand;
import frc.robot.commands.xbox.XboxRumbleCommand;
import frc.robot.commands.xbox.XboxToggleRumbleCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.Limelight;
import frc.robot.utils.XboxDPad;
import frc.robot.utils.XboxTrigger;
import frc.robot.utils.XboxDPad.Direction;
import frc.robot.utils.XboxTrigger.Hand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private CommandScheduler commandScheduler;
    public Joystick driverStationJoystick;
    private XboxController xboxController; 
    private XboxTrigger rightTrigger; 
    private XboxTrigger leftTrigger;
    public boolean isDriverStation;
    
    public DrivetrainSubsystem drivetrainSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public HopperSubsystem hopperSubsystem;
    public Limelight limelight;
    public PinchRollerSubsystem pinchRollerSubsystem;
    public HoodSubsystem hoodSubsystem;
    public TurretSubsystem turretSubsystem;
    public LEDSubsystem ledSubsystem;
    public ShiftingGearSubsystem shiftingGearSubsystem;
    private HashMap<Integer, CommandBase> buttonMap;
    
    public static boolean isRumbling = false; // for xbox
    
    public RobotContainer(){
        commandScheduler = CommandScheduler.getInstance();
            
        isDriverStation = !(DriverStation.getJoystickIsXbox(0) || DriverStation.getJoystickIsXbox(1)); 

        shooterSubsystem = new ShooterSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        drivetrainSubsystem = new DrivetrainSubsystem();
        hopperSubsystem = new HopperSubsystem();
        limelight = new Limelight();
        pinchRollerSubsystem = new PinchRollerSubsystem();
        hoodSubsystem = new HoodSubsystem();
        turretSubsystem = new TurretSubsystem();
        ledSubsystem = new LEDSubsystem();
        shiftingGearSubsystem = new ShiftingGearSubsystem();
        buttonMap = new HashMap<>();
        
        switch (drivetrainSubsystem.getDriveMode()) {
            case TANK:
                drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.tankDrive(getLeftY(), getRightY()), drivetrainSubsystem));
                break;
            case CHEEZY:
                drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.cheezyDrive(getLeftY(), getRightX()), drivetrainSubsystem));
                break;
            case ARCADE:
                drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.arcadeDrive(getLeftY(), getRightX()), drivetrainSubsystem));
                break;
        }



        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // POWER PORT ROBOT CONTROLS
        // setJoystickButtonWhenPressed(driverStationJoystick, 1, new TurnToTargetPIDCommand(turretSubsystem));
        // setJoystickButtonWhenPressed(driverStationJoystick, 2, new ToggleIntakePistonCommand(intakeSubsystem));
        // setJoystickButtonWhileHeld(driverStationJoystick, 3, new AllInCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
        // setJoystickButtonWhenPressed(driverStationJoystick, 4, new ToggleShooterCommand(shooterSubsystem));
        // setJoystickButtonWhileHeld(driverStationJoystick, 5, new AllOutCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
        // setJoystickButtonWhenPressed(driverStationJoystick, 6, new StopShooterCommand(shooterSubsystem));
        // setJoystickButtonWhileHeld(driverStationJoystick, 7, new TestHoodCommand(hoodSubsystem, .1));
        // setJoystickButtonWhileHeld(driverStationJoystick, 8, new TestHoodCommand(hoodSubsystem, -.1));
        // setJoystickButtonWhileHeld(driverStationJoystick, 9, new TurretTestCommand(turretSubsystem, -0.2)); //left
        // setJoystickButtonWhileHeld(driverStationJoystick, 10, new TurretTestCommand(turretSubsystem, 0.2)); //right
        // setJoystickButtonWhileHeld(driverStationJoystick, 11, new IntakeInCommand(intakeSubsystem));
        
        if(isDriverStation){
            driverStationJoystick = new Joystick(OIConstants.DRIVER_STATION_JOY);
            setJoystickButtonWhenPressed(driverStationJoystick, 1, new TurnToTargetPIDCommand(turretSubsystem));
            setJoystickButtonWhileHeld(driverStationJoystick, 2, new AllInCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
            setJoystickButtonWhenPressed(driverStationJoystick, 3, new ToggleShooterCommand(shooterSubsystem));
            setJoystickButtonWhileHeld(driverStationJoystick, 4, new TestHoodCommand(hoodSubsystem, HoodConstants.HOOD_SPEED)); //up
            setJoystickButtonWhileHeld(driverStationJoystick, 5, new TurretTestCommand(turretSubsystem, TurretConstants.TURRET_SPEED)); //right
            
            setJoystickButtonWhileHeld(driverStationJoystick, 6, new IntakeInCommand(intakeSubsystem));
            setJoystickButtonWhileHeld(driverStationJoystick, 7, new AllOutCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
            //button 8 not used
            setJoystickButtonWhileHeld(driverStationJoystick, 9, new TestHoodCommand(hoodSubsystem, -HoodConstants.HOOD_SPEED)); //down
            setJoystickButtonWhileHeld(driverStationJoystick, 10, new TurretTestCommand(turretSubsystem, -TurretConstants.TURRET_SPEED)); //left
            
            setJoystickButtonWhenPressed(driverStationJoystick, 11, new ToggleShiftingCommand(shiftingGearSubsystem, drivetrainSubsystem));
            setJoystickButtonWhenPressed(driverStationJoystick, 12, new ToggleIntakePistonCommand(intakeSubsystem));
        } else {
            /*  
                Allowed buttons:
                kA, kB, kBack, kBumperLeft, kBumperRight, kStart, kStickLeft, kStickRight, kX, kY (and triggers)
            */

            int port = (DriverStation.getJoystickIsXbox(0)) ? 0 : 1;
            xboxController = new XboxController(port); 

            new RunCommand(() -> new XboxRumbleCommand(xboxController, OIConstants.XBOX_RUMBLE_INTENSITY));

            rightTrigger = new XboxTrigger(xboxController, Hand.RIGHT);
            leftTrigger = new XboxTrigger(xboxController, Hand.LEFT);

            setXboxButtonWhenPressed(xboxController, Button.kLeftStick, new ToggleShiftingCommand(shiftingGearSubsystem, drivetrainSubsystem), false);
            setXboxButtonWhenPressed(xboxController, Button.kRightStick, new ToggleIntakePistonCommand(intakeSubsystem), false);
            
            setXboxTriggerWhileHeld(Hand.RIGHT, new AllInCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
            setXboxButtonWhileHeld(xboxController, Button.kRightBumper, new IntakeInCommand(intakeSubsystem));
            setXboxTriggerWhileHeld(Hand.LEFT, new AllOutCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
            setXboxButtonWhileHeld(xboxController, Button.kLeftBumper, new IntakeOutCommand(intakeSubsystem));
            
            setXboxDPadWhileHeld(Direction.LEFT, new TurretTestCommand(turretSubsystem, -TurretConstants.TURRET_SPEED));//left
            setXboxDPadWhileHeld(Direction.RIGHT, new TurretTestCommand(turretSubsystem, TurretConstants.TURRET_SPEED));//right
            
            setXboxButtonWhenPressed(xboxController, Button.kA, new TurnToTargetPIDCommand(turretSubsystem), false);
            setXboxButtonWhenPressed(xboxController, Button.kB, new ToggleShooterCommand(shooterSubsystem), true);
            setXboxButtonWhileHeld(xboxController, Button.kY, new TestHoodCommand(hoodSubsystem, HoodConstants.HOOD_SPEED));// up
            setXboxButtonWhileHeld(xboxController, Button.kX, new TestHoodCommand(hoodSubsystem, -HoodConstants.HOOD_SPEED));// down
        }
    }

    public double getLeftY() {
        return (isDriverStation) ? -driverStationJoystick.getRawAxis(0) : -xboxController.getLeftY();
    }

    public double getLeftX() {
        return (isDriverStation) ? driverStationJoystick.getRawAxis(1) : -xboxController.getLeftX();
    }

    public double getRightY() {
        return (isDriverStation) ? -driverStationJoystick.getRawAxis(2) : -xboxController.getRightY();
    }

    public double getRightX() {
        return (isDriverStation) ? driverStationJoystick.getRawAxis(3) : -xboxController.getRightX();
    }

    public HashMap<Integer, CommandBase> getButtonMap() {
        return buttonMap;
    }

    private void setJoystickButtonWhenPressed(Joystick joystick, int button, CommandBase command) {
        new JoystickButton(joystick, button).whenPressed(command);
        buttonMap.put(button, command);
    }

    private void setJoystickButtonWhileHeld(Joystick joystick, int button, CommandBase command) {
        new JoystickButton(joystick, button).whileHeld(command);
        buttonMap.put(button, command);
    }

    // Xbox controller equivalents
    private void setXboxButtonWhenPressed(XboxController xboxController, XboxController.Button button, CommandBase command, boolean withRumble) {
        if(withRumble){
            command = command.beforeStarting(new XboxToggleRumbleCommand());
        }
        new JoystickButton(xboxController, button.value).whenPressed(command);
    }

    private void setXboxButtonWhileHeld(XboxController xboxController, XboxController.Button button, CommandBase command) {
        new JoystickButton(xboxController, button.value).whileHeld(command);
    }

    private void setXboxTriggerWhenPressed(Hand triggerSide, CommandBase command){
        if(triggerSide == Hand.LEFT){ 
            leftTrigger.whenActive(command);
        } else {
            rightTrigger.whenActive(command);
        }
    }

    private void setXboxTriggerWhileHeld(Hand triggerSide, CommandBase command){
        if(triggerSide == Hand.LEFT){ 
            leftTrigger.whileActiveContinuous(command);
        } else {
            rightTrigger.whileActiveContinuous(command);
        }
    }

    private void setXboxDPadWhenPressed(Direction direction, CommandBase command) {
        new XboxDPad(xboxController, direction).whenPressed(command);
    }

    private void setXboxDPadWhileHeld(Direction direction, CommandBase command) {
        new XboxDPad(xboxController, direction).whileHeld(command);
    }

    public void updateIsDriverStation(){
        boolean prev = isDriverStation;
        isDriverStation = !(DriverStation.getJoystickIsXbox(0) || DriverStation.getJoystickIsXbox(1));
        if (prev == isDriverStation) {
            return;
        } else {
            commandScheduler.clearButtons();
            configureButtonBindings();
        }
    }

    public Command getAutonomousCommand() {

        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    AutoConstants.ksVolts,
                    AutoConstants.kvVoltSecondsPerMeters,
                    AutoConstants.kaVoltSecondsSquaredPerMeter
                ),
            AutoConstants.kinematics,
            10.5);
        
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            // Units.inchesToMeters(DriveConstants.MAX_SPEED_INCHES_PER_SEC),
            // Units.inchesToMeters(DriveConstants.MAX_ACCEL_INCHES_PER_SEC2))
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        Trajectory customTrajectory = customTrajectory();

        // Trajectory customTrajectory = TrajectoryGenerator.generateTrajectory(
        //     // Start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // Pass through these two interior waypoints, making an 's' curve path
        //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //     // End 3 meters straight ahead of where we started, facing forward
        //     new Pose2d(3, 0, new Rotation2d(0)),
        //     // Pass config
        //     config);

        RamseteCommand ramseteCommand =     
            new RamseteCommand(
                customTrajectory,
                drivetrainSubsystem::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    AutoConstants.ksVolts,
                    AutoConstants.kvVoltSecondsPerMeters,
                    AutoConstants.kaVoltSecondsSquaredPerMeter
                ),
                AutoConstants.kinematics,
                drivetrainSubsystem::getDifferentialDriveSpeeds,
                new PIDController(AutoConstants.kPDriveVel, 0, 0),
                new PIDController(AutoConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrainSubsystem::tankDriveVolts,
                drivetrainSubsystem
            );

        // Reset odometry to the starting pose of the trajectory.
        drivetrainSubsystem.resetOdometry(customTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));
    }

    public Trajectory customTrajectory(){
        String trajectoryJSON = "paths/basic_curve.wpilib.json";
        Trajectory trajectory;
        // Trajectory trajectory = new Trajectory();
        System.out.println("PathWeaverTest initialized");
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          return trajectory;
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
          return null;
        }
    }

    // public boolean getButtonStatus(Joystick joystick, int button) {
    //     return driverStationJoystick.getRawButton(button);
    // }

    public static void toggleIsRumble(){
        isRumbling = !isRumbling;
    }
}
