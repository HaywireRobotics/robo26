// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;
import  edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class HID_Keyturn {
    private final GenericHID key;

    public HID_Keyturn(int port){
        key = new GenericHID(port);
    }

    public boolean GetKeyState(){
        for (int i = 1; i <= 12; i++) {
            if (key.getRawButton(i)) {
                return true;
            }
        }
        return false;
    }
}
