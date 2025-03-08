package frc.robot.commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.*;
//import frc.robot.commands.*;
//import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.ElevatorSubsytem;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;
import frc.robot.subsystems.Elevator.EndEffectorSubsytem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
//import frc.robot.subsystems.intake.AlgaeIntakeSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class IntakeSequence extends Command{
    ElevatorSubsytem m_ElevatorSubsytem;
    EndEffectorSubsytem m_EndEffectorSubsytem;
    //AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem;
    CoralIntakeSubsystem m_CoralIntakeSubsystem;

    public WaitCommand waitwait(double time) {
        return new WaitCommand(time);
    }

    public InstantCommand set_Height(HeightLevels height) {
        return new InstantCommand(()-> m_ElevatorSubsytem.setHeightWithEnum(height));
    }

    public Command incrementHeight(int incrementBy) {
        return new InstantCommand(() -> m_ElevatorSubsytem.incrementByStage(incrementBy));
    }

    public InstantCommand endEffectorAngle(double angle) {
        return new InstantCommand(()->m_EndEffectorSubsytem.setRotation(angle));
    }

    // public InstantCommand algaeArmAngle(double point){
    //     return new InstantCommand(()->m_AlgaeIntakeSubsystem.setSetpoint(point));
    //     //angle is 0.1 or 0.12
    // }
    // public InstantCommand algaeWheelSpin(double speed){
    //     return new InstantCommand(()->m_AlgaeIntakeSubsystem.turnIntakeWheels(speed)); 
    // }
    
    public InstantCommand coralArmAngle(double point){
        return new InstantCommand(()->m_CoralIntakeSubsystem.setSetpoint(point)); 
    }

    public InstantCommand coralWheelSpin(double speed){
        return new InstantCommand(()->m_CoralIntakeSubsystem.intakeSpinWheels(speed)); 
    }
    public IntakeSequence(ElevatorSubsytem m_ElevatorSubsytem,EndEffectorSubsytem m_EndEffectorSubsytem) {
        this.m_ElevatorSubsytem = m_ElevatorSubsytem;
        this.m_EndEffectorSubsytem = m_EndEffectorSubsytem;
/*
 * 1 - turn on elevator motors to raise elevator to a specific height
 * 2 - wait some amount of time
 * 3- have the endeffector rotate to pick up the coral
 * 4 - maybe add a wait command
 * 5  - raise the elevator a little bit so that the endeffector doesn't hit the base
 * 6 - rotate the endeffector back to completly vertical
 * 7 - possible wait command
 * 8 - lower the elevator to resting position
 */
        //addCommands(set_Height(1), waitwait(1),endEffectorAngle(180),waitwait(1.5),set_Height(.5),endEffectorAngle(0),set_Height(0));
    }
    //This sequence is to intake an algae from off of the floor
    //Could possibly be used to outake as well as there is just a different angle that could be a negative wheel speed
    // public IntakeSequence(AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem){
    //     this.m_AlgaeIntakeSubsystem = m_AlgaeIntakeSubsystem;
    //     /*
    //      * 1 - move algae are to a specific angle 
    //      * 2 - spin wheels to intake algae
    //      */
    //     //addCommand(algaeArmAngle(.1),waitwait(0.5),algaeWheelSpin(1));
    // }
    //This command sequence is to intake coral from off the floor not any other time
    public IntakeSequence(CoralIntakeSubsystem m_CoralIntakeSubsystem){
        this.m_CoralIntakeSubsystem = m_CoralIntakeSubsystem;
        /*
         * 1 - move coral are to a specific angle
         * 2 - spin wheels to intake 
         */
        //addCommand(coralArmAngle(.1),waitwait(0.5),coraleWheelSpin(1));
    }
}
