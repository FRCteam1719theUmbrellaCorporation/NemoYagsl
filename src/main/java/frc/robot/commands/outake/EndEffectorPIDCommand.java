// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outake;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.EndefectorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsytem;
import frc.robot.subsystems.Elevator.EndEffectorSubsytem;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorPIDCommand extends Command {
  private EndEffectorSubsytem m_EndEffector;
  ElevatorSubsytem m_ElevatorSubsytem;
  boolean setElevator;
  boolean ElevatorMovesFirst;
  PIDController m_endEffPID;
  HeightLevels currentHeight;
  /** Creates a new ArmRotatePID. */
  public EndEffectorPIDCommand(EndEffectorSubsytem endeff, ElevatorSubsytem mElevatorSubsytem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endeff, mElevatorSubsytem);
    m_EndEffector = endeff;
    m_ElevatorSubsytem = mElevatorSubsytem;
    setElevator = true;
    ElevatorMovesFirst = true;
    m_endEffPID = endeff.getPID();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveBoth(currentHeight.ZERO);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double output = MathUtil.clamp(m_endEffPID.calculate(m_EndEffector.doubleMeasurement()), EndefectorConstants.MIN_SPEED, EndefectorConstants.MAX_SPEED);

    // sets the elevator when the arm is near it's setpoint
    if (setElevator && MathUtil.isNear(m_EndEffector.getSetpoint(), m_EndEffector.doubleMeasurement(), 0.1)) {
      m_ElevatorSubsytem.setHeightWithEnum(m_EndEffector.getHeight());
      setElevator = false;
    } 

    if (!ElevatorMovesFirst) {
      m_EndEffector.setRotation(output);
      // System.out.println("endeff output: " + output);
    } else if (MathUtil.isNear(m_ElevatorSubsytem.getSetPoint(), m_ElevatorSubsytem.doubleMeasurement(), 1)){
      ElevatorMovesFirst = false;
    }
  }

  public Command moveBoth(HeightLevels level) {

    return new InstantCommand(() -> {
      if (level.numVal() > m_ElevatorSubsytem.getSetPoint()) {
        m_ElevatorSubsytem.setHeightWithEnum(level);
        ElevatorMovesFirst = true;
        setElevator = false;
      } else {
        setElevator = true;
        ElevatorMovesFirst = false;
      }

      // if (m_EndEffector.getHeight().numVal() < EndefectorConstants.INTAKE_POS_ELEVATORPOS_MAX && m_ElevatorSubsytem.doubleMeasurement() < EndefectorConstants.INTAKE_POS_ELEVATORPOS_MAX) {

      // }
      m_EndEffector.setHeight(level);
    });
  }

  public BooleanSupplier isAtPos() {
    return () -> MathUtil.isNear(m_EndEffector.getSetpoint(), m_EndEffector.doubleMeasurement(), 0.1) && MathUtil.isNear(m_ElevatorSubsytem.getSetPoint(), m_ElevatorSubsytem.doubleMeasurement(), 4);
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
