package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.CargoTrapSubsystem;

public class AutoCargoIntakeCommand extends CommandBase {
    private CargoTrapSubsystem trap;
    private double speed;

    public AutoCargoIntakeCommand(double speed) {
        addRequirements(Robot.cargoTrap);
        trap = Robot.cargoTrap;
        this.speed = speed;
    }

    @Override
   public void initialize() {
        trap.deployTrap();        
    }

    @Override
    public void execute() {
        trap.setIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        trap.retractTrap();
        trap.setIntake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}