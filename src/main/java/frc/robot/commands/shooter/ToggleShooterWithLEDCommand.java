package frc.robot.commands.shooter;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooterWithLEDCommand extends ToggleShooterCommand {
	LEDSubsystem ledSubsystem;
	int rpmError = 500;

	public ToggleShooterWithLEDCommand(ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
		super(shooterSubsystem);
		this.ledSubsystem = ledSubsystem;
		addRequirements(shooterSubsystem, ledSubsystem);
	}

	@Override
	public void initialize() {
		for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
    		ledSubsystem.setRGB(i, 255, 0, 0);
    	}
    	ledSubsystem.sendData();
	}

	@Override
	public void execute() {
		super.execute();
		// rm 1600
		// setpoint 1550
		// 1601 - 50
		if (this.shooterSubsystem.getFlywheelRPM() - rpmError <= this.shooterSubsystem.setPoint || this.shooterSubsystem.getFlywheelRPM() + rpmError >= this.shooterSubsystem.setPoint) {
			// within error of setpoint
			for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
    			ledSubsystem.setRGB(i, 0, 255, 0);
    		}
    		ledSubsystem.sendData();
		} else {
			// not near setpoint
			for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
    			ledSubsystem.setRGB(i, 255, 0, 0);
    		}
    		ledSubsystem.sendData();
		}
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
	}

	@Override
	public boolean isFinished() {
		return super.isFinished();
	}
}
