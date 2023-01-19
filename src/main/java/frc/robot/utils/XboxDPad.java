package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxDPad extends Trigger {
	private XboxController xboxController;
	private Direction direction;
	
	public XboxDPad(XboxController xboxController, Direction direction) {
		super(() -> {
			int value = xboxController.getPOV();
			return Math.abs(value - direction.value) <= 45;
		});
		this.xboxController = xboxController;
		this.direction = direction;
	};
	
	public static enum Direction {
		UP(0), RIGHT(90), DOWN(180), LEFT(270);

		int value;

		private Direction(int direction) {
			value = direction;
		}
	}
}