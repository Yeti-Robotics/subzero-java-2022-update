// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.xbox;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class XboxToggleRumbleCommand extends InstantCommand {
  public XboxToggleRumbleCommand() {}

  @Override
  public void initialize() {
    RobotContainer.toggleIsRumble();
  }
}