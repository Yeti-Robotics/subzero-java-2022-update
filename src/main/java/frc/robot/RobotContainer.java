package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ToggleShiftingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShiftingGearSubsystem;



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
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private DriverStation driverStation;
    private CommandScheduler commandScheduler;
    public Joystick driverStationJoystick;
    private XboxController xboxController; 
    public boolean isDriverStation;

    public DriveSubsystem drivetrainSubsystem;
    public ShiftingGearSubsystem shiftingGearSubsystem;
    private HashMap<Integer, CommandBase> buttonMap;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driverStation = DriverStation.getInstance();
        commandScheduler = CommandScheduler.getInstance();
        

        drivetrainSubsystem = new DriveSubsystem();
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
            driverStationJoystick = new Joystick(Constants.OIConstants.DRIVER_STATION_JOY);
            setJoystickButtonWhenPressed(driverStationJoystick, 11, new ToggleShiftingCommand(shiftingGearSubsystem, drivetrainSubsystem));
        } else {
            /*  
                Allowed buttons:
                kA, kB, kBack, kBumperLeft, kBumperRight, kStart, kStickLeft, kStickRight, kX, kY (and triggers)
            */
            }
    }

    public double getLeftY() {
        return -driverStationJoystick.getRawAxis(0);
    }

    public double getLeftX() {
        return driverStationJoystick.getRawAxis(1);
    }

    public double getRightY() {
        return -driverStationJoystick.getRawAxis(2);
    }

    public double getRightX() {
        return driverStationJoystick.getRawAxis(3);
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
    private void setXboxButtonWhenPressed(XboxController xboxController, XboxController.Button button, CommandBase command) {
        new JoystickButton(xboxController, button.value).whenPressed(command);
    }

    private void setXboxButtonWhileHeld(XboxController xboxController, XboxController.Button button, CommandBase command) {
        new JoystickButton(xboxController, button.value).whileHeld(command);
    }

    
    public void updateIsDriverStation(){
        boolean prev = isDriverStation;
       // isDriverStation = !driverStation.getJoystickIsXbox(OIConstants.XBOX_PORT);
        if (prev == isDriverStation) {
            return;
        } else {
            commandScheduler.clearButtons();
            configureButtonBindings();
        }
    }

    public Command getAutonomousCommand() {
        // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.DriveConstants.ksVolts,
            Constants.DriveConstants.kvVoltSecondsPerInch,
            Constants.DriveConstants.kaVoltSecondsSquaredPerInch),
        Constants.DriveConstants.kDriveKinematics,
        10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedInchesPerSecond,
                    Constants.AutoConstants.kMaxAccelerationInchesPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        RamseteCommand ramseteCommand = 
            new RamseteCommand(
                exampleTrajectory,
                m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    Constants.DriveConstants.ksVolts,
                    Constants.DriveConstants.kvVoltSecondsPerInch,
                    Constants.DriveConstants.kaVoltSecondsSquaredPerInch),
                Constants.DriveConstants.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }


    public boolean getButtonStatus(Joystick joystick, int button) {
        return driverStationJoystick.getRawButton(button);
    }
}