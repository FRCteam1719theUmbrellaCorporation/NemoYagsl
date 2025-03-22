// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outake;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator.ElevatorSubsytem;
import frc.robot.subsystems.Elevator.EndEffectorSubsytem;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorPIDCommand extends Command {
  private EndEffectorSubsytem m_EndEffector;
  private boolean setElevator;
  private boolean ElevatorMovesFirst;
  private ElevatorSubsytem m_ElevatorSubsytem;
  private HeightLevels currentHeight;

  /** Creates a new ArmRotatePID. */
  public EndEffectorPIDCommand(EndEffectorSubsytem endeff, ElevatorSubsytem mElevatorSubsytem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endeff, mElevatorSubsytem);
    m_EndEffector = endeff;
    m_ElevatorSubsytem = mElevatorSubsytem;
    setElevator = true;
    ElevatorMovesFirst = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveBoth(HeightLevels.ZERO);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // sets the elevator when the arm is near it's setpoint
    if (setElevator && MathUtil.isNear(m_EndEffector.getSetpoint(), m_EndEffector.doubleMeasurement(), 0.1)) {
      m_ElevatorSubsytem.setHeightWithEnum(m_EndEffector.getHeight());
      setElevator = false;
    } 

    if (!ElevatorMovesFirst && !setElevator) {
      m_EndEffector.setSetpoint(m_EndEffector.getHeight().armVal());
      // System.out.println("endeff output: " + output);
    } else if (MathUtil.isNear(m_ElevatorSubsytem.getSetPoint(), m_ElevatorSubsytem.doubleMeasurement(), 1)){
      ElevatorMovesFirst = false;
    }
  }

  /**
   * Allows the user to move the arm and the elevator at the same time. 
   * The prefered direction of the arm will be determined by the enum attatched
   * 
   * @param level: height level that you want the elevator and arm to move to
   * @return Command that begins the arm and elevator moving in different directions
   */
  public Command moveBoth(HeightLevels level) {
    return moveBoth(level, level.preferedDirection());
  }

  /**
   * Allows the user to move the arm and the elevator at the same time.
   * This specific command allows the arm to move to the closest spot regaurdless of if the enum wants it to
   * 
   * @param level: height level that you want the elevator and arm to move to
   * @return Command that begins the arm and elevator moving in different directions
   */
  public Command moveBothNoDirection(HeightLevels level) {
    return moveBoth(level, null);
  }

  /**
   * Allows the user to move the arm and the elevator at the same time. 
   * A passed in value will be applied for the direction of the arm
   * 
   * @param level: height level that you want the elevator and arm to move to
   * @param explicitDirection: Direction the endeffector will move in. + is left, - is right. null is whatever is closest
   * @return Command that begins the arm and elevator moving in different directions
   */
  public Command moveBoth(HeightLevels level, Boolean explicitDirection) {
    currentHeight = level;
    
    return new InstantCommand(() -> {
      // elevator moves first when going up
      if (level.numVal() > m_ElevatorSubsytem.getSetPoint()) {
        m_ElevatorSubsytem.setHeightWithEnum(level);
        ElevatorMovesFirst = true;
        setElevator = true;

      // enedeffector moves first when going down
      } else {
        setElevator = true;
        ElevatorMovesFirst = false;
      }

      // if (m_EndEffector.getHeight().numVal() < EndefectorConstants.INTAKE_POS_ELEVATORPOS_MAX && m_ElevatorSubsytem.doubleMeasurement() < EndefectorConstants.INTAKE_POS_ELEVATORPOS_MAX) {

      // }
      m_EndEffector.setHeight(level);
      m_EndEffector.setDirection(explicitDirection);
    });
  }
  /**
   * Check if the endeffector and arm are at their required height setpoints
   * 
   * @param heightToCheck: Height level to compare to 
   * @return boolean supplier that asserts that both elevator and endeffector are at that currentpos
   */
  public BooleanSupplier isAtPos() {
    return () -> MathUtil.isNear(m_EndEffector.getSetpoint(), m_EndEffector.doubleMeasurement(), 0.1) && MathUtil.isNear(m_ElevatorSubsytem.getSetPoint(), m_ElevatorSubsytem.doubleMeasurement(), 1);
  }

  /**
   * Check if the endeffector and arm are at a specified location
   * 
   * @param heightToCheck: Height level to compare to 
   * @return boolean supplier that asserts that both elevator and endeffector are at that currentpos
   */
  public BooleanSupplier comparePosition(HeightLevels heightToCheck) {
    return () -> MathUtil.isNear(heightToCheck.armVal(), m_EndEffector.doubleMeasurement(), 0.1) && MathUtil.isNear(heightToCheck.numVal(), m_ElevatorSubsytem.doubleMeasurement(), 1);
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
