package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CargoTrapSubsystem;

public class AutoCargoIntakeCommand extends Command {
    private CargoTrapSubsystem trap;
    private double speed;

    public AutoCargoIntakeCommand(double speed) {
        requires(Robot.cargoTrap);
        trap = Robot.cargoTrap;
        this.speed = speed;
    }

    @Override
    protected void initialize() {
        trap.deployTrap();        
    }

    @Override
    protected void execute() {
        trap.setIntake(speed);
    }

    @Override
    protected void end() {
        trap.retractTrap();
        trap.setIntake(0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}