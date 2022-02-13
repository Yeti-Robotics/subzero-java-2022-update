package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.XboxSubsystem;

public class IntakeInCommandWithRumble extends IntakeInCommand {
	private final XboxSubsystem xboxSubsystem;
	public IntakeInCommandWithRumble(IntakeSubsystem intakeSubsystem, XboxSubsystem xboxSubsystem) {
		super(intakeSubsystem);
		this.xboxSubsystem = xboxSubsystem;
		addRequirements(intakeSubsystem, xboxSubsystem);
	}

	@Override
	public void initialize() {
		xboxSubsystem.rumble(0.5);
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
		xboxSubsystem.stopRumble();
	}
}
