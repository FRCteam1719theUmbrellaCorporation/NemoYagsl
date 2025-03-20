// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import utils.Reef.Level; // i will end it

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralCommand extends SequentialCommandGroup {
  private static EndEffectorPIDCommand m_cmd;
  private static SwerveSubsystem m_swerve; // to be used to move the robot backwards

  // these will be the desired height levels to move the robot to
  public static volatile HeightLevels height1;
  public static volatile HeightLevels height2;

  /**
   * Defines the 
   * 
   * @param cmd: controller command for both the arm and elevator movements
   * @param swerve: Swervedrive subsystem, which is currently unused
   */
  public PlaceCoralCommand(EndEffectorPIDCommand cmd, SwerveSubsystem swerve) {
    m_cmd = cmd;
    m_swerve = swerve;
    System.out.println(Robot.reefLevel);
  }

  /**
   * Arm is limited to 180 degree motion, and places at l2
   * 
   * WARNING MAY BE UNSAFE DUE TO NEW CHANGES :)
   * 
   * @return Sequential command for placing on L2
   */
  @Deprecated
  public static SequentialCommandGroup placeL2() {
    return new SequentialCommandGroup(
        m_cmd.moveBoth(HeightLevels.LOW_PRE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.LOW),
        new WaitUntilCommand(m_cmd.isAtPos()),
        new WaitCommand(2),
        m_cmd.moveBoth(HeightLevels.INTAKE_UP),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_PRE_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos())
      );
  }

  // test
  public static SequentialCommandGroup placeL2ThenIntake(SequentialCommandGroup intakeCommand) {
    return new SequentialCommandGroup(

      intakeCommand,
      m_cmd.moveBoth(HeightLevels.INTAKE_PRE_DOWN),
      new WaitUntilCommand(m_cmd.isAtPos()),
      m_cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN)
      );
  }

  // // OLD
  // public static SequentialCommandGroup placeL3() {
  //   return new SequentialCommandGroup(
  //       m_cmd.moveBoth(HeightLevels.Middle_PRE),
  //       new WaitUntilCommand(m_cmd.isAtPos()),
  //       m_cmd.moveBoth(HeightLevels.MIDDLE),
  //       new WaitUntilCommand(m_cmd.isAtPos()),
  //       new WaitCommand(2),
  //       m_cmd.moveBoth(HeightLevels.ZERO),
  //       new WaitUntilCommand(m_cmd.isAtPos())
  //     );
  // }

  /**
   * Arm is limited to 180 degree motion, and places at l3
   * 
   * WARNING MAY BE UNSAFE DUE TO NEW CHANGES :)
   * 
   * @return Sequential command for placing on L3
   */
  @Deprecated
  public static SequentialCommandGroup placeL3() {
    return new SequentialCommandGroup(
        m_cmd.moveBoth(HeightLevels.Middle_PRE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.MIDDLE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        new WaitCommand(2),
        m_cmd.moveBoth(HeightLevels.INTAKE_UP),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_PRE_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos())
      );
  }

  /**
   * Arm is limited to 180 degree motion, and places at l4
   * 
   * WARNING MAY BE UNSAFE DUE TO NEW CHANGES :)
   * 
   * @return Sequential command for placing on L4
   */
  @Deprecated
  public static SequentialCommandGroup placeL4() {
    return new SequentialCommandGroup(
        m_cmd.moveBoth(HeightLevels.HIGH_PRE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.HIGH),
        new WaitUntilCommand(m_cmd.isAtPos()),
        new WaitCommand(2),
        m_cmd.moveBoth(HeightLevels.INTAKE_UP),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_PRE_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos())
        // m_cmd.moveBoth(HeightLevels.ZERO),
        // new WaitUntilCommand(m_cmd.isAtPos())
      );
  }
  
  /**
   * placeL2, however the arm is allowed to move in a 360 degree moton
   * 
   * @return Sequential command for placing on L2
   */
  public static SequentialCommandGroup l2CommandFlip() {
    return new SequentialCommandGroup(
      m_cmd.moveBoth(HeightLevels.INTAKE_FLIP_AROUND),
      new WaitUntilCommand(m_cmd.isAtPos()),
      new WaitCommand(0.1),
      m_cmd.moveBoth(HeightLevels.LOW_PRE),
      new WaitUntilCommand(m_cmd.isAtPos()),
      m_cmd.moveBoth(HeightLevels.LOW),
      new WaitUntilCommand(m_cmd.isAtPos()),
      new WaitCommand(2),
      m_cmd.moveBoth(HeightLevels.INTAKE_FLIP_BACK),
      new WaitUntilCommand(m_cmd.isAtPos()),
      m_cmd.moveBoth(HeightLevels.INTAKE_FLIP_TO_DOWN),
      new WaitUntilCommand(m_cmd.isAtPos()),
      m_cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN),
      new WaitUntilCommand(m_cmd.isAtPos())
      // m_cmd.moveBoth(HeightLevels.INTAKE_UP),
      // new WaitUntilCommand(m_cmd.isAtPos()),
      // m_cmd.moveBoth(HeightLevels.INTAKE_PRE_DOWN),
      // new WaitUntilCommand(m_cmd.isAtPos()),
      // m_cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN),
      // new WaitUntilCommand(m_cmd.isAtPos())
      );
  }


  /**
   * placeL3, however the arm is allowed to move in a 360 degree moton
   * 
   * @return Sequential command for placing on L3
   */
  public static SequentialCommandGroup l3CommandFlip() {
    return new SequentialCommandGroup(
      m_cmd.moveBoth(HeightLevels.INTAKE_FLIP_AROUND),
      new WaitUntilCommand(m_cmd.isAtPos()),
      new WaitCommand(0.1),
        m_cmd.moveBoth(HeightLevels.Middle_PRE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.MIDDLE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        new WaitCommand(2),
        m_cmd.moveBoth(HeightLevels.INTAKE_UP),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_PRE_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos())
      );
  }

  /**
   * placeL4, however the arm is allowed to move in a 360 degree moton
   * 
   * @return Sequential command for placing on l4
   */
  public static SequentialCommandGroup l4CommandFlip() {
    return new SequentialCommandGroup(
      m_cmd.moveBoth(HeightLevels.INTAKE_FLIP_AROUND),
      new WaitUntilCommand(m_cmd.isAtPos()),
      new WaitCommand(0.1),

      m_cmd.moveBoth(HeightLevels.HIGH_PRE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.HIGH),
        new WaitUntilCommand(m_cmd.isAtPos()),
        new WaitCommand(2),
        m_cmd.moveBoth(HeightLevels.INTAKE_UP),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_PRE_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos())
    );
  }

  /**
   * Safely resets the arm to it's Pre intake position
   * 
   * @return Sequential Command for moving itself to correct spot
   */
  public static SequentialCommandGroup resetArm() {
    return new SequentialCommandGroup(
        m_cmd.moveBoth(HeightLevels.INTAKE_FLIP_AROUND),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_FLIP_BACK),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN),
        new WaitUntilCommand(m_cmd.isAtPos())
        // m_cmd.moveBoth(HeightLevels.ZERO),
        // new WaitUntilCommand(m_cmd.isAtPos())
      );
  }

    // public static SequentialCommandGroup placeAt(Level rLevel) {
  
  //   // System.out.println(RobotContainer.level);

  //   if (height1 == null) {
  //     PlaceCoralCommand.height1 = HeightLevels.LOW_PRE;
  //     PlaceCoralCommand.height2 = HeightLevels.LOW;
  //   }
    
  //   // switch(rLevel) {
  //   //   case L2:
  //   //     PlaceCoralCommand.height1 = HeightLevels.LOW_PRE;
  //   //     PlaceCoralCommand.height2 = HeightLevels.LOW;

  //   //     break;
  //   //   case L3:
  //   //     PlaceCoralCommand.height1 = HeightLevels.Middle_PRE;
  //   //     PlaceCoralCommand.height2 = HeightLevels.MIDDLE;
  //   //     break;
  //   //   default:
  //   //     PlaceCoralCommand.height1 = HeightLevels.HIGH_PRE;
  //   //     PlaceCoralCommand.height2 = HeightLevels.HIGH;
  //   //     break;
  //   // }

  //   return new SequentialCommandGroup(
  //       m_cmd.moveBoth(PlaceCoralCommand.height1),
  //       new WaitUntilCommand(m_cmd.isAtPos()),
  //       m_cmd.moveBoth(PlaceCoralCommand.height2),
  //       new WaitUntilCommand(m_cmd.isAtPos()),
  //       new WaitCommand(2),

  //       m_cmd.moveBoth(HeightLevels.ZERO),
  //       new WaitUntilCommand(m_cmd.isAtPos())
        
  //     );
  // } 

  // i'd rather kms then write this code again


}
