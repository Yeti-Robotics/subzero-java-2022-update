// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class AuroraLEDCommand extends CommandBase {
  /** Creates a new AuroraLEDCommand. */
  private LEDSubsystem ledSubsystem;
  public AuroraLEDCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int j = 0; j < 85; j += 91) {
      for (int i = 0 + j; i < 23 + j; i++) {
        ledSubsystem.setRGB(i, 20, 250, 50);
      }
      for (int i = 23 + j; i < 46 + j; i++) {
        ledSubsystem.setRGB(i, 5, 100, 220);
      }
      for (int i = 46 + j; i < 68 + j; i++) {
        ledSubsystem.setRGB(i, 120, 10, 150);
      }
      for (int i = 68 + j; i < 91 + j; i++) {
        ledSubsystem.setRGB(i, 200, 50, 100);
      }
    }
    ledSubsystem.sendData();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
