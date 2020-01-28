package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class OdometryVisionAlign extends CommandBase {
    private Drivetrain drivetrain;
    private Turret turret;
    private double kP;
    private double setpoint;
    private double coordSystemTheta;
    private double coordX;
    private double coordY;
    private final double netHeight = 62.25; //look this up from design
    private double deadReckonX;
    private double deadReckonY;
    private double angleAdjust;
    private DifferentialDriveOdometry m_odometry;
    private Rotation2d heading;
    private Translation2d deadReckoning;
    private double driveAngle;
    private Pose2d pose;

    public OdometryVisionAlign(Drivetrain drivetrain, Turret turret){
        addRequirements(drivetrain, turret);
        this.drivetrain = drivetrain;
        this.turret = turret;
        kP = 0.03; //need to tune

    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders(0,0);
        drivetrain.resetGyro();
        heading = new Rotation2d(); //zeroed
        m_odometry = new DifferentialDriveOdometry(heading);
        coordY = (netHeight)/Math.tan(Robot.lastTyVal);
        coordX = coordY*Math.tan(turret.getPosition());
        angleAdjust = 0;
    }

    @Override
    public void execute() {

        driveAngle = drivetrain.getAngle(); //is this in degrees or radians??? useMath.toDeg if needed
        pose = m_odometry.update(Rotation2d.fromDegrees(driveAngle), drivetrain.getLeftEncoderPos(), drivetrain.getRightEncoderPos());
        deadReckoning = pose.getTranslation();
        deadReckonX = deadReckoning.getX(); //use odometry for these
        deadReckonY = deadReckoning.getY();

        coordSystemTheta = Math.tan((coordY - deadReckonY) / (coordX - deadReckonX));

        setpoint = Robot.lastAlignedHeading + (drivetrain.getAngle() * -1) + coordSystemTheta;

        angleAdjust = kP * setpoint;

        turret.setPWM(angleAdjust);

    }

    public double getCoordX(){
        return coordX;
    }

    public double getCoordY(){
        return coordY;
    }
}
