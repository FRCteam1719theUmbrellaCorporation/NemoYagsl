package frc.robot.commands.SequenceCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.ElevatorSubsytem;
import frc.robot.subsystems.Elevator.EndEffectorSubsytem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class IntakeSequence extends Command{
    ElevatorSubsytem m_ElevatorSubsytem;
    EndEffectorSubsytem m_EndEffectorSubsytem;

    public WaitCommand waitwait(double time) {
        return new WaitCommand(time);
    }

    public InstantCommand set_Height(double height) {
        return new InstantCommand(()-> m_ElevatorSubsytem.setGoal(height));
    }

    public InstantCommand endEffectorAngle(double angle) {
        return new InstantCommand(()->m_EndEffectorSubsytem.setRotation(angle));
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
    

}
