
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Landmark;
//import frc.robot.Landmarks;

public class PlotLandmarks extends SubsystemBase{
    private final Field2d m_field = new Field2d(); 
    public PlotLandmarks() {

        

        // Plot all landmarks as "objects" on the field
        //m_field.setRobotPose(pose);;
        m_field.getObject("Hub").setPose(Landmark.HUB.get());
        m_field.getObject("Tower").setPose(Landmark.TOWER.get());
        m_field.getObject("Outpost").setPose(Landmark.OUTPOST.get());
        m_field.getObject("RightBump").setPose(Landmark.RIGHT_BUMP.get());
        m_field.getObject("LeftBump").setPose(Landmark.LEFT_BUMP.get());
        m_field.getObject("RightTrench").setPose(Landmark.RIGHT_TRENCH.get());
        m_field.getObject("LeftTrench").setPose(Landmark.LEFT_TRENCH.get());
        m_field.getObject("Depot").setPose(Landmark.DEPOT.get());
        // Discontinued
        SmartDashboard.putData("Field", m_field);
    }
     public void updateRobotPose(Pose2d pose) {
        FieldObject2d targetPose = m_field.getRobotObject();
        targetPose.setPose(pose);
    }
 
}
