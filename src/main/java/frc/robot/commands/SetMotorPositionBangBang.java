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
  double error;

  /** Creates a new SetMotorPositionBangBang. */
  public SetMotorPositionBangBang(Intake intakeSubsystem, double desiredPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.desiredPosition = desiredPosition;
    this.error = Constants.CONTROL_LOOP_ERROR_DEGREES;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentPos = intakeSubsystem.getEncoderPosition();
    double motorSpeed = Constants.BANG_BANG_CONTROL_SPEED;

    if (!isWithinError(currentPos)) { // not there state
      if (currentPos > this.desiredPosition) {
        intakeSubsystem.setSpinSpeed(-motorSpeed);
      } else if (currentPos < this.desiredPosition) {
        intakeSubsystem.setSpinSpeed(motorSpeed);
      }
    } else if (isWithinError(currentPos)) { // there state
      intakeSubsystem.setSpinSpeed(0);
    }
  }

  private boolean isWithinError(double position) {
    double upperLimit = this.desiredPosition + this.error;
    double lowerLimit = this.desiredPosition - this.error;
    if (position > upperLimit || position < lowerLimit) {
      return false;
    }
    return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setSpinSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
