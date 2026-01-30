package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Landmarks;

public class PlotLandmarks extends SubsystemBase{
    private final Field2d m_field = new Field2d();
    
    public void PlotLandmarks() {
        SmartDashboard.putData("Field", m_field);

        // Plot all landmarks as "objects" on the field
        m_field.getObject("Hub").setPose(Landmarks.hubPosition("center"));
        m_field.getObject("Tower").setPose(Landmarks.towerPosition());
        m_field.getObject("Outpost").setPose(Landmarks.outpostPosition());
        m_field.getObject("RightBump").setPose(Landmarks.rightBumpPosition());
        m_field.getObject("LeftBump").setPose(Landmarks.leftBumpPosition());
        m_field.getObject("RightTrench").setPose(Landmarks.rightTrenchPosition());
        m_field.getObject("LeftTrench").setPose(Landmarks.leftTrenchPosition());
        m_field.getRobotPose(m_drive.getRobotPose());
        // Discontinued
    }
 
}
