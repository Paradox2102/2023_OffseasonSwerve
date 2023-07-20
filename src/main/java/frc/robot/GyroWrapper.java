// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Add your docs here. */
public class GyroWrapper implements Gyro {

    PigeonIMU m_gyro;

    public GyroWrapper(PigeonIMU gyro) {
        m_gyro = gyro;
    }

    @Override
    public void close() throws Exception {
        
    }

    @Override
    public void calibrate() {
        m_gyro.enterCalibrationMode(CalibrationMode.Temperature);
    }

    @Override
    public void reset() {
        m_gyro.setYaw(0);
    }

    public void resetYaw(double angle) {
        m_gyro.setYaw(angle); //in degrees 
    }

    @Override
    public double getAngle() {
        double[] ypr = new double[3];
        m_gyro.getYawPitchRoll(ypr);
        return ypr[0];
    }

    @Override
    public double getRate() {
        double[] xyz = new double[3];
        m_gyro.getRawGyro(xyz);
        return xyz[2];
    }
}