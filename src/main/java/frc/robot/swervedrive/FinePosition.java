package frc.robot.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveModule;

public class FinePosition extends Command {
    SwerveSubsystem swerve;
    Pose2d targetPose;

    public FinePosition(SwerveSubsystem swerve, Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        var currentPose = swerve.getPose();
        var xDist = targetPose.getX() - currentPose.getX();
        var yDist = targetPose.getY() - currentPose.getY();
        var deltaAngle = currentPose.getRotation().minus(targetPose.getRotation());
        //right now, we are using the distance as our speed in m/s.  Probably needs to be tuned...
        var targetSpeed = new Translation2d(xDist, yDist).times(8);
        //swerve.drive(targetSpeed, deltaAngle.getRadians() * 0, true);
        SmartDashboard.putNumber("xError", xDist);
        SmartDashboard.putNumber("yError", yDist);
        SmartDashboard.putNumber("deltaError", deltaAngle.getRadians());

        SwerveModuleState state = new SwerveModuleState(0,Rotation2d.fromDegrees(rotation));
        swerve.setModuleStates(new SwerveModuleState[] {state, state, state, state}, true);
        rotation +=5;
    }
    double rotation=0;
}
