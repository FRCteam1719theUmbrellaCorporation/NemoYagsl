package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePIDCommand extends Command {
    private final IntakeSubsystem intake;
    private final PIDController m_ArmAngleManager;
    private static DoubleSupplier SETPOINTANGLE;
    private final DutyCycleEncoder m_ANGLE_ENCODER;

    public IntakePIDCommand(IntakeSubsystem intake, PIDController m_ArmAngleManager, DoubleSupplier SETPOINTANGLE, DutyCycleEncoder m_ANGLE_ENCODER) {
        addRequirements(intake);
        this.intake = intake;
        this.m_ArmAngleManager = m_ArmAngleManager;
        this.SETPOINTANGLE = SETPOINTANGLE;
        this.m_ANGLE_ENCODER = m_ANGLE_ENCODER;

    }
    
    @Override
    public void initialize(){
        m_ArmAngleManager.setSetpoint(SETPOINTANGLE.getAsDouble());
    }

    @Override
    public void execute() {
        m_ArmAngleManager.setSetpoint(intake.isIntaking() ? Constants.IntakeDetails.intakePos : SETPOINTANGLE.getAsDouble());

        intake.setRotation(m_ArmAngleManager.calculate(m_ArmAngleManager.calculate(m_ANGLE_ENCODER.get())));
    }

    @Override
    public void end(boolean interupt) {
        // this should always run sorry gang
    }
}
