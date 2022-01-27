// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.xbox;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class XboxRumbleCommand extends CommandBase {
  private XboxController xboxController;
  private double rumbleIntensity;

  public XboxRumbleCommand(XboxController xboxController, double rumbleIntensity) {
    this.xboxController = xboxController;
    this.rumbleIntensity = rumbleIntensity;
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute(){
    if(RobotContainer.isRumbling){
      xboxController.setRumble(RumbleType.kLeftRumble, rumbleIntensity);
      xboxController.setRumble(RumbleType.kRightRumble, rumbleIntensity);
    } else {
      xboxController.setRumble(RumbleType.kLeftRumble, 0.0);
      xboxController.setRumble(RumbleType.kRightRumble, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    xboxController.setRumble(RumbleType.kLeftRumble, 0.0);
    xboxController.setRumble(RumbleType.kRightRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}