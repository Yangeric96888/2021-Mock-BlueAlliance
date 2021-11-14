// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private final DriveTrain _driveTrain;
  private final Joystick _joystick;
  public ArcadeDrive(DriveTrain dt, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    _joystick = joystick;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _driveTrain.arcadeDrive(-0.8 * _joystick.getRawAxis(Constants.JoystickAxis.YAxis),
                    0.8 * _joystick.getRawAxis(Constants.JoystickAxis.XAxis));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
