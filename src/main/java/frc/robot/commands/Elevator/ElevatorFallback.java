// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.jni.CANBusJNI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator.ElevatorSubsytem;
import frc.robot.subsystems.Elevator.EndEffectorSubsytem;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorFallback extends Command {
  private Boolean elevatororEndeff = null; // if true, is elevator, else is endeff
  ElevatorSubsytem m_elevator;
  EndEffectorSubsytem m_endeffector;
  /** Creates a new ElevatorFallback. */
  public ElevatorFallback(ElevatorSubsytem ele, EndEffectorSubsytem endeff) {
    addRequirements(ele, endeff);
    m_endeffector = endeff;
    m_elevator = ele;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_elevator.isConnected()) {
      elevatororEndeff = true;
    } else if (!m_endeffector.isConnected()) {
      elevatororEndeff = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // this handles a disconnect
  public Command handle() {
    if (elevatororEndeff == true) {
      return new SequentialCommandGroup(new InstantCommand(()->m_endeffector.setSetpoint(0)));
    } else if (elevatororEndeff == false) {
      if (m_elevator.getSetPoint() < HeightLevels.INTAKE_PRE_DOWN.numVal()) {
        return new SequentialCommandGroup(
          new InstantCommand(()->m_elevator.setSetpoint(HeightLevels.INTAKE_PRE_DOWN.numVal())),
          new WaitUntilCommand(()->MathUtil.isNear(HeightLevels.INTAKE_PRE_DOWN.numVal(), m_elevator.doubleMeasurement(), 4)),
          new InstantCommand(()->m_elevator.setSetpoint(HeightLevels.INTAKE.numVal()))
        );
      } else {
        return new InstantCommand(()->m_elevator.setSetpoint(HeightLevels.INTAKE.numVal()));
      }
    } else {
      return Commands.none(); // idk what to put here :)
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
