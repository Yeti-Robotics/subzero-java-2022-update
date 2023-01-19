// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

public class XboxTrigger extends Trigger {
    public static enum Hand {
        LEFT, RIGHT;
    }

    private XboxController xboxController;
    private Hand hand;

    public XboxTrigger(XboxController xboxController, Hand hand){
        super(() -> {
            double raw = (hand == Hand.LEFT) ? xboxController.getLeftTriggerAxis() : xboxController.getRightTriggerAxis();
            return raw >= OIConstants.TRIGGER_THRESHOLD;
        });
        this.hand = hand;
        this.xboxController = xboxController; 
    }
}
