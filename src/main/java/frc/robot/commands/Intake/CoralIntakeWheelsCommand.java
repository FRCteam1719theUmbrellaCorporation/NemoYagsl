package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
// import frc.robot.Constants;

public class CoralIntakeWheelsCommand extends Command {
    CoralIntakeSubsystem m_CoralIntakeSubsystem;

    public CoralIntakeWheelsCommand(CoralIntakeSubsystem input) {
        m_CoralIntakeSubsystem = input;
    }
    public InstantCommand turnMotor(double speed) {
        return new InstantCommand(()-> 
            m_CoralIntakeSubsystem.intakeSpinWheels(speed)
        );
    }

    // hold coral halfway
    public SequentialCommandGroup halfIntake() {
        return new SequentialCommandGroup(
            turnMotor(CoralArmConstants.coral_intake_floor_speed),
            new WaitUntilCommand(m_CoralIntakeSubsystem.hasCoral()).withTimeout(4),
            turnMotor(0)
            );
    }

    public InstantCommand stopMotors(){
        return new InstantCommand(()->
            m_CoralIntakeSubsystem.turnOffIntakeWheel()
        );
    }
}
