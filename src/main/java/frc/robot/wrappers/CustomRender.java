// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;

/** Add your docs here. */
public class CustomRender {
    private CvSource m_source;

    private int m_width;
    private int m_height;

    private Mat m_screen;

    public CustomRender(String name, int width, int height) {
        m_width = width;
        m_height = height;
        m_source = CameraServer.putVideo(name, width, height);

        m_screen = new Mat(width, height, CvType.CV_64FC3);
    }

    public void render() {
        for (int x = 0; x < m_width; x++) {
            for (int y = 0; y < m_height; y++) {
                m_screen.put(x, y, ((double) x) / m_width, ((double) y) / m_height, 0);
            }
        }

        m_source.putFrame(m_screen);
    }
}
