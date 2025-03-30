// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.falconfury.frc2025.commands;

import org.falconfury.frc2025.RobotState;
import org.falconfury.frc2025.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTeleop extends Command {
  // Subsystems
  private final Drive drive_;
  private final RobotState robot_state_;

  // Playstation controller
  private final CommandPS5Controller controller_;

  /** Creates a new DriveTeleop. */
  public DriveTeleop(Drive drive, RobotState robot_state, CommandPS5Controller controller) {
    drive_ = drive;
    robot_state_ = robot_state;
    controller_ = controller;

    addRequirements(drive_);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = controller_.getLeftY() * Constants.kTranslationMultiplier;
  }

  // Constants Class
  public static final class Constants {
    public static final double kTranslationMultiplier = 2.5;
    public static final double kRotationMultiplier = Math.PI;
  }
}
