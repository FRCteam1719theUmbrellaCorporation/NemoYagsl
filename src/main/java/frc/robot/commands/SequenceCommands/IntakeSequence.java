package frc.robot.commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsytem;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;
import frc.robot.subsystems.Elevator.EndEffectorSubsytem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


// this file feels a lot like #define for c funcs

public class IntakeSequence extends Command{
    // subsystems
    ElevatorSubsytem m_ElevatorSubsytem;
    EndEffectorSubsytem m_EndEffectorSubsytem;
    CoralIntakeSubsystem m_CoralIntakeSubsystem;

    /**
     * Wait command
     * @param time: time in seconds
     * @return
     */
    public WaitCommand waitwait(double time) {
        return new WaitCommand(time);
    }

    /**
     * sets a height level for the elevator
     * 
     * @param height
     * @return
     */
    public InstantCommand set_Height(HeightLevels height) {
        return new InstantCommand(()-> m_ElevatorSubsytem.setHeightWithEnum(height));
    }

    /**
     * Moves elevator up by 1 spot
     */
    public Command incrementHeight(int incrementBy) {
        return new InstantCommand(() -> m_ElevatorSubsytem.incrementByStage(incrementBy));
    }

    /**
     * Sets the endeffector angle
     * 
     * @param angle
     * @return
     */
    public InstantCommand endEffectorAngle(double angle) {
        return new InstantCommand(()->m_EndEffectorSubsytem.setRotation(angle));
    }

    /**
     * Sets the arm angle for coral subsystem
     */
    public InstantCommand coralArmAngle(double point){
        return new InstantCommand(()->m_CoralIntakeSubsystem.setSetpoint(point)); 
    }

    /**
     * Sets the speed at which the coral intake moves at
     * 
     * @param speed
     * @return
     */
    public InstantCommand coralWheelSpin(double speed){
        return new InstantCommand(()->m_CoralIntakeSubsystem.intakeSpinWheels(speed)); 
    }

    /**
     * Defines the constants for the class
     * 
     * @param m_ElevatorSubsytem
     * @param m_EndEffectorSubsytem
     */
    public IntakeSequence(ElevatorSubsytem m_ElevatorSubsytem,EndEffectorSubsytem m_EndEffectorSubsytem) {
        this.m_ElevatorSubsytem = m_ElevatorSubsytem;
        this.m_EndEffectorSubsytem = m_EndEffectorSubsytem;
    }
}
