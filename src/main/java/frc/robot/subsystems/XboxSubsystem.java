package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.XController;

public class XboxSubsystem extends SubsystemBase {
	private XController controller;
	private boolean isDriverStation = !(DriverStation.getJoystickIsXbox(0) || DriverStation.getJoystickIsXbox(1)); 

	public XboxSubsystem() {
		int port = (DriverStation.getJoystickIsXbox(0)) ? 0 : 1;
		controller = new XController(port);
	}

	public XController getController() {
		return this.controller;
	}

	public void rumble(double power) {
		if (!isDriverStation) {
			controller.setRumble(RumbleType.kLeftRumble, power);
			controller.setRumble(RumbleType.kRightRumble, power);
		}
	}

	public void rumbleRight(double power) {
		if (!isDriverStation) {
			controller.setRumble(RumbleType.kRightRumble, power);
		}
	}

	public void rumbleLeft(double power) {
		if (!isDriverStation) {
			controller.setRumble(RumbleType.kLeftRumble, power);
		}
	}

	public void stopRumble() {
		if (!isDriverStation) {
			controller.setRumble(RumbleType.kLeftRumble, 0);
			controller.setRumble(RumbleType.kRightRumble, 0);
		}
	}
}
