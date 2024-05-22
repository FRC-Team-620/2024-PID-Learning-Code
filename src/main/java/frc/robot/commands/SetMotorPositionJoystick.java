// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class SetMotorPositionJoystick extends Command {
  Intake intakeSubsystem;
  XboxController controller;
  double desiredPosition;

  /** Creates a new SetMotorPositionJoystick. */
  public SetMotorPositionJoystick(Intake intakeSubsystem, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.controller = controller;
    this.desiredPosition = this.intakeSubsystem.getEncoderPosition();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get current sensor position
    double currentPos = intakeSubsystem.getEncoderPosition();

    // calculate error
    double error = this.desiredPosition - currentPos;

    // calculate proportional controller output
    double output = MathUtil.clamp(
        Constants.ARM_K_proportional * error,
        -Constants.ARM_MAX_OUTPUT,
        Constants.ARM_MAX_OUTPUT);

    // set motor output
    intakeSubsystem.setSpinSpeed(output);
  }

  // returns joystick angle as degrees (-180 - 180)
  private double getJoystickAsAngle() {
    // code here
    return 0;
  }

  // should return the current angle converted to be within -180 - 180
  private double getConvertedCurrentPosition() {
    // code here
    return 0;
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
