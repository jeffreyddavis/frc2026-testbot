package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FlipUtil;
import frc.robot.subsystems.vision.Vision;



public class RobotHealth extends SubsystemBase  {
    @AutoLogOutput
    public boolean PoseHealthy = true;
    @AutoLogOutput
    public boolean PoseJump = false;
    @AutoLogOutput
    public boolean YawDisagreement = false;
    @AutoLogOutput
    public boolean timeout = false;
    @AutoLogOutput
    public boolean fieldReady = false;
    @AutoLogOutput
    public boolean fieldOptimal = false;

    @AutoLogOutput
    public boolean inAllianceZone = true; // start on your side of the field.
    @AutoLogOutput
    public boolean inNeutralZone = false;
    @AutoLogOutput
    public boolean inOpponentZone = false;
    @AutoLogOutput
    public boolean inTrenchZones = false;

    @AutoLogOutput
    public boolean hoodDangerNearTrench = false;

    private Drivetrain m_Drivetrain;
    private QuestNaviman m_QuestNaviman;
    private Vision m_Vision;

    @AutoLogOutput
    private Pose2d lastPose2d;
    private Pose2d newPose2d;

    private double unhealthyUntil = 0;

    private int fieldStatus = 0;              // 0=red, 1=yellow, 2=green
    private int desiredFieldStatus = 0;
    private double statusStableUntil = 0.0;

    private static final double FIELD_STATUS_DEBOUNCE = 0.4; // seconds



    @AutoLogOutput
    public String fieldStatusText;
    
    @Override
    public void periodic() {
        newPose2d = m_Drivetrain.getPose(); // get fused position from all sources
        PoseJump = poseHasJumped();
        
        YawDisagreement = YawDisagreement();
        
        timeout = poseTimeout();

        if (PoseJump || YawDisagreement || timeout) {
            unhealthyUntil = Timer.getFPGATimestamp() + 0.25;
        }

        PoseHealthy = Timer.getFPGATimestamp() > unhealthyUntil;

        fieldReady = (PoseHealthy && m_Vision.camerasWithPoseCount >= 2);
        fieldOptimal = (PoseHealthy && m_Vision.camerasWithPoseCount >= 3);


        determineFieldState();

        lastPose2d = newPose2d; // prep for next periodic;
    }

    public void updateZones() {
        Pose2d pose = m_Drivetrain.getPose();
        double poseX = FlipUtil.applyX(pose.getX());

        double myZone = FlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
        double neutralZoneNear = FlipUtil.applyX(FieldConstants.LinesVertical.neutralZoneNear);
        double neutralZoneFar = FlipUtil.applyX(FieldConstants.LinesVertical.neutralZoneNear);
        double oppZone = FlipUtil.applyX(FieldConstants.LinesVertical.oppAllianceZone);

        inAllianceZone = (poseX < myZone);
        inNeutralZone = (poseX > neutralZoneNear && poseX <= neutralZoneFar);
        inTrenchZones = (poseX >= myZone && poseX <= neutralZoneNear ) 
            || (poseX < oppZone && poseX > neutralZoneFar ) ;
        inOpponentZone = (poseX > oppZone);

        Translation2d trench1 = FlipUtil.apply(FieldConstants.LeftBump.nearLeftCorner);
        Translation2d trench2 = FlipUtil.apply(FieldConstants.LeftBump.nearRightCorner);
        Translation2d trench3 = FlipUtil.apply(FieldConstants.LeftBump.nearLeftCorner);
        Translation2d trench4 = FlipUtil.apply(FieldConstants.LeftBump.nearRightCorner);

        hoodDangerNearTrench = (pose.getTranslation().getDistance(trench1) < Constants.TrenchDangerDistance ||
            pose.getTranslation().getDistance(trench2) < Constants.TrenchDangerDistance ||
            pose.getTranslation().getDistance(trench3) < Constants.TrenchDangerDistance ||
            pose.getTranslation().getDistance(trench4) < Constants.TrenchDangerDistance
            );

        


    }

    public void determineFieldState() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Determine desired state
        if (!PoseHealthy || m_Vision.camerasWithPoseCount <= 1) {
            desiredFieldStatus = 0; // RED
        } else if (m_Vision.camerasWithPoseCount == 2) {
            desiredFieldStatus = 1; // YELLOW
        } else {
            desiredFieldStatus = 2; // GREEN
        }
        
        // Debounce logic
        if (desiredFieldStatus != fieldStatus) {
            // Only transition if stable long enough
            if (now > statusStableUntil) {
                fieldStatus = desiredFieldStatus;
                statusStableUntil = now + FIELD_STATUS_DEBOUNCE;
            }
        } else {
            // Extend stability window
            statusStableUntil = now + FIELD_STATUS_DEBOUNCE;
        }
        
        switch (fieldStatus) {
            case 0: fieldStatusText = "NOT READY"; break;
            case 1: fieldStatusText = "READY"; break;
            case 2: fieldStatusText = "OPTIMAL"; break;
        }
        

    }

    @AutoLogOutput
    public int FieldStatus() {
        return fieldStatus;
    }

    public RobotHealth(Drivetrain drivetrain, QuestNaviman questNaviman, Vision vision) {
        m_Drivetrain = drivetrain;
        m_QuestNaviman = questNaviman;
        m_Vision = vision;
        lastPose2d = drivetrain.getPose();

    }

    private boolean poseTimeout() {
        return !m_Drivetrain.pigeon.isConnected() 
            && !m_QuestNaviman.questNav.isConnected() 
            && !m_QuestNaviman.questNav.isTracking();
    }

    private boolean YawDisagreement() {
        double yawError = Math.abs(
            m_Drivetrain.getHeading()
                .minus(m_QuestNaviman.questRobotPose2d.getRotation())
                .getDegrees()
        );

        return yawError > Constants.YawWarningTolerance;

    }

    private boolean poseHasJumped() {
        double deltaX = newPose2d.getX() - lastPose2d.getX();
        double deltaY = newPose2d.getY() - lastPose2d.getY();
        double distanceJump = Math.hypot(deltaX, deltaY);

        double maxPhysicallyPossible = Constants.MAX_SPEED * .02  * 2; // .02 is ROBORIO 50Hz.

        return distanceJump > maxPhysicallyPossible;
    }
    
}
