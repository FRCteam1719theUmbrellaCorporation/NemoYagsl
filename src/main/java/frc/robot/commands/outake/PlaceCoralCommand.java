// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralCommand extends SequentialCommandGroup {
  private static EndEffectorPIDCommand m_cmd;
  private static SwerveSubsystem m_swerve;

  public PlaceCoralCommand(EndEffectorPIDCommand cmd, SwerveSubsystem swerve) {
    m_cmd = cmd;
    m_swerve = swerve;
  }

  public static SequentialCommandGroup placeAt() {
    HeightLevels height1;
    HeightLevels height2;


    switch(RobotContainer.level) {
      case L2:
        height1 = HeightLevels.LOW_PRE;
        height2 = HeightLevels.LOW;
        break;
      case L3:
        height1 = HeightLevels.Middle_PRE;
        height2 = HeightLevels.MIDDLE;
        break;
      default:
        height1 = HeightLevels.HIGH_PRE;
        height2 = HeightLevels.HIGH;
        break;
    }

    return new SequentialCommandGroup(
        // cmd.moveBoth(HeightLevels.INTAKE_UP),
        // new WaitUntilCommand(cmd.isAtPos()),
        m_cmd.moveBoth(height1),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(height2),
        new WaitUntilCommand(m_cmd.isAtPos()),
        new WaitCommand(2),
        // m_swerve.driveToPose(null),
        m_cmd.moveBoth(HeightLevels.ZERO),
        new WaitUntilCommand(m_cmd.isAtPos())
        
      );
  } 
}
