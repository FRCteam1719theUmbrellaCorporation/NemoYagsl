package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;

public class CoralIntakeWheelsCommand extends Command {
    CoralIntakeSubsystem m_CoralIntakeSubsystem;

    /**
     * @param input: coral subsystem
     */
    public CoralIntakeWheelsCommand(CoralIntakeSubsystem input) {
        m_CoralIntakeSubsystem = input;
    }

    /**
     * Instant command that moves the coral wheels
     * 
     * @param speed: turns the coral wheels to this speed
     */
    public InstantCommand turnMotor(double speed) {
        return new InstantCommand(()-> 
            m_CoralIntakeSubsystem.intakeSpinWheels(speed)
        );
    }

    /**
     * Intakes until the command is stopped, or a coral is intaken 
     * 
     * @return: the command group
     */
    public SequentialCommandGroup halfIntake() {
        return new SequentialCommandGroup(
            turnMotor(CoralArmConstants.coral_intake_floor_speed)
            .until(m_CoralIntakeSubsystem.hasCoral())
            .andThen(stopMotors())
            // new WaitUntilCommand(),
            // turnMotor(0)
            );
    }

    /**
     * Fully intakes a coral into the robot
     * 
     * @return: the command group
     */
    public SequentialCommandGroup fullIntake() {
        return new SequentialCommandGroup(
            turnMotor(CoralArmConstants.coral_intake_floor_speed),
            new WaitUntilCommand(m_CoralIntakeSubsystem.hasCoral()),
            turnMotor(CoralArmConstants.coral_intake_floor_speed_limited)
            );
    }

    /**
     * Turns off the intake wheels
     * 
     * @return: the command group
     */
    public InstantCommand stopMotors(){
        return new InstantCommand(()->
            m_CoralIntakeSubsystem.turnOffIntakeWheel()
        );
    }
}
