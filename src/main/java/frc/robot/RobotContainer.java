// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AllInCommand;
import frc.robot.commands.AllInShootCommand;
import frc.robot.commands.AllOutCommand;
import frc.robot.commands.autonav.BarrelRacingCommandGroup;
import frc.robot.commands.autonav.BouncePathCommandGroup;
import frc.robot.commands.autonav.SlalomCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.ToggleShiftingCommand;
import frc.robot.commands.drivetrain.DriveForDistanceLowPIDCommand;
import frc.robot.commands.drivetrain.DriveForDistanceProfiledPIDCommand;
import frc.robot.commands.drivetrain.ToggleDriveModeCommand;
import frc.robot.commands.drivetrain.TurnForAnglePIDCommand;
import frc.robot.commands.groups.AimTurretAndHoodCommandGroup;
import frc.robot.commands.groups.FireBallCommandGroup;
import frc.robot.commands.hood.SetCalcHoodAngleCommand;
import frc.robot.commands.hood.TestHoodCommand;
import frc.robot.commands.hopper.HopperInCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeInCommandWithRumble;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.ToggleIntakePistonCommand;
import frc.robot.commands.pinchroller.PinchRollerInCommand;
import frc.robot.commands.replay.InitiateRecordingCommand;
import frc.robot.commands.replay.PlayRecordingCommand;
import frc.robot.commands.replay.TerminateAndSaveRecordingCommand;
import frc.robot.commands.shooter.SpinToRPMCommand;
import frc.robot.commands.shooter.StopFullIntakeCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.commands.shooter.ShootingCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.commands.turret.CalibrateTurretCommand;
import frc.robot.commands.turret.TurnToAnglePIDCommand;
import frc.robot.commands.turret.TurnToTargetPIDCommand;

import frc.robot.commands.turret.TurretTestCommand;
import frc.robot.commands.xbox.XboxRumbleCommand;
import frc.robot.commands.xbox.XboxToggleRumbleCommand;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DrivetrainSubsystem.DriveMode;
import frc.robot.utils.Limelight;
import frc.robot.utils.XController;
import frc.robot.utils.XboxDPad;
import frc.robot.utils.XboxTrigger;
import frc.robot.utils.XboxDPad.Direction;
import frc.robot.utils.XboxTrigger.Hand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.HashMap;


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
    private XboxSubsystem xboxSubsystem;
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

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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
        xboxSubsystem = new XboxSubsystem();
        buttonMap = new HashMap<>();

        
        
        switch (drivetrainSubsystem.getDriveMode()) {
            case TANK:
                drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.tankDrive(getLeftY(), getRightY()), drivetrainSubsystem));
                break;
            case CHEEZY:
                drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.cheezyDrive(getRightX(), -getLeftY()), drivetrainSubsystem));
                break;
            case ARCADE:
                drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.arcadeDrive(getRightX(), -getLeftY()), drivetrainSubsystem));
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

            setXboxButtonWhenPressed(xboxSubsystem.getController(), Button.kLeftStick, new ToggleShiftingCommand(shiftingGearSubsystem, drivetrainSubsystem));
            setXboxButtonWhenPressed(xboxSubsystem.getController(), Button.kRightStick, new ToggleIntakePistonCommand(intakeSubsystem));
            
            xboxSubsystem.getController().setTriggerWhileHeld(Hand.RIGHT, new AllInCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
            setXboxButtonWhileHeld(xboxSubsystem.getController(), Button.kRightBumper, new IntakeInCommandWithRumble(intakeSubsystem, xboxSubsystem));
            xboxSubsystem.getController().setTriggerWhileHeld(Hand.LEFT, new AllOutCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
            setXboxButtonWhileHeld(xboxSubsystem.getController(), Button.kLeftBumper, new IntakeOutCommand(intakeSubsystem));
            
           // xboxSubsystem.getController().setDPadWhileHeld(Direction.LEFT, new TurretTestCommand(turretSubsystem, -TurretConstants.TURRET_SPEED));//left
           // xboxSubsystem.getController().setDPadWhileHeld(Direction.RIGHT, new TurretTestCommand(turretSubsystem, TurretConstants.TURRET_SPEED));//right
            
            setXboxButtonWhenPressed(xboxSubsystem.getController(), Button.kA, new TurnToTargetPIDCommand(turretSubsystem));
            setXboxButtonWhenPressed(xboxSubsystem.getController(), Button.kB, new ToggleShooterCommand(shooterSubsystem));
            setXboxButtonWhileHeld(xboxSubsystem.getController(), Button.kY, new TestHoodCommand(hoodSubsystem, HoodConstants.HOOD_SPEED));// up
            setXboxButtonWhileHeld(xboxSubsystem.getController(), Button.kX, new TestHoodCommand(hoodSubsystem, -HoodConstants.HOOD_SPEED));// down
        }
    }

    public double getLeftY() {
        return (isDriverStation) ? -driverStationJoystick.getRawAxis(0) : (xboxSubsystem.getController().getLeftY());
    }

    public double getLeftX() {
        return (isDriverStation) ? driverStationJoystick.getRawAxis(1) : (-xboxSubsystem.getController().getLeftX());
    }

    public double getRightY() {
        return (isDriverStation) ? -driverStationJoystick.getRawAxis(2) : (-xboxSubsystem.getController().getRightY());
    }

    public double getRightX() {
        return (isDriverStation) ? driverStationJoystick.getRawAxis(3) : (-xboxSubsystem.getController().getRightX());
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
    private void setXboxButtonWhenPressed(XboxController xboxController, Button button, CommandBase command) {
        new JoystickButton(xboxController, button.value).whenPressed(command);
    }

    private void setXboxButtonWhileHeld(XboxController xboxController, Button button, CommandBase command) {
        new JoystickButton(xboxController, button.value).whileHeld(command);
    }

    private void setXboxDPadWhenPressed(Direction direction, CommandBase command) {
        new XboxDPad(xboxSubsystem.getController(), direction).whenPressed(command);
    }

    private void setXboxDPadWhileHeld(Direction direction, CommandBase command) {
        new XboxDPad(xboxSubsystem.getController(), direction).whileHeld(command);
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
        Command command = new PlayRecordingCommand("1616845434755recording.txt", drivetrainSubsystem);
        return command;
    }

    public boolean getButtonStatus(Joystick joystick, int button) {
        return driverStationJoystick.getRawButton(button);
    }

    public static void toggleIsRumble(){
        isRumbling = !isRumbling;
    }
}
