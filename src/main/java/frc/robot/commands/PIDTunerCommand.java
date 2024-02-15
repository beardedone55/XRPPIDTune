package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;

public class PIDTunerCommand extends Command {
    
    private RomiDrivetrain drivetrain;
    private double desiredSpeed = 5.0; //inches /er second


    public PIDTunerCommand(RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Desired Speed", ()->desiredSpeed, (a)->desiredSpeed=a);
    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        drivetrain.setSpeeds(desiredSpeed, desiredSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeeds(0,0);
    }
}
