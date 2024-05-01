// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class SetMotorPositionBangBang extends Command {
  Intake intakeSubsystem;
  double desiredPosition;

  /** Creates a new SetMotorPositionBangBang. */
  public SetMotorPositionBangBang(Intake intakeSubsystem, double desiredPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.desiredPosition = desiredPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intakeSubsystem.getEncoderPosition() != this.desiredPosition) { // not there state
      intakeSubsystem.setSpinSpeed(-Constants.SIMPLE_MOTOR_SPEED);
    } else if (intakeSubsystem.getEncoderPosition() == this.desiredPosition) { // there state
      intakeSubsystem.setSpinSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
