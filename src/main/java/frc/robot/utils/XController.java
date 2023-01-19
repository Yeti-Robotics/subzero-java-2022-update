package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.XboxDPad.Direction;
import frc.robot.utils.XboxTrigger.Hand;

public class XController extends XboxController {
	public XController(int port) {
		super(port);
	}

	public void setTriggerWhenPressed(Hand triggerSide, CommandBase command){
        if(triggerSide == Hand.LEFT){ 
            new XboxTrigger(this, triggerSide).onTrue(command);
        } else {
            new XboxTrigger(this, triggerSide).onTrue(command);
        }
    }

	public void setTriggerWhileHeld(Hand triggerSide, CommandBase command){
        if(triggerSide == Hand.LEFT) { 
            new XboxTrigger(this, triggerSide).whileTrue(command);
        } else {
            new XboxTrigger(this, triggerSide).whileTrue(command);
        }
    }

	public void setDPadWhenPressed(Direction direction, CommandBase command) {
        new XboxDPad(this, direction).onTrue(command);
    }

	public void setDPadWhileHeld(Direction direction, CommandBase command) {
        new XboxDPad(this, direction).whileTrue(command);
    }
}
