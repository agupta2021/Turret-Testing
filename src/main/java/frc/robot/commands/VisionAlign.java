package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class VisionAlign extends CommandBase{
    private Turret turret;
    private double kP;
    private double tx;
    private double angleAdjust;


    public VisionAlign(Turret turret){
        addRequirements(turret); //interruptible is set in scheduling
        this.turret = turret;
    }

    @Override
    public void initialize() {
        kP = 0.03;
        angleAdjust = 0;
    }

    @Override
    public void execute() {
        tx = turret.getLimelight().getTargetHorizontalOffset(0);
        angleAdjust = tx * kP;
        turret.setPWM(angleAdjust);
        Robot.lastAlignedHeading = turret.getPosition();
        Robot.lastTyVal = turret.getLimelight().getTargetVerticalOffset(0); //NOTE: these 2 values only update if hasValidTarget()
    }

    @Override
    public void end(boolean interrupted) {

    }
}
