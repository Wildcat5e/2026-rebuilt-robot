
package frc.robot.subsystems;

import com.andymark.jni.AM_CAN_Color_Sensor;
import com.andymark.jni.AM_CAN_Color_Sensor.AM_ColorSensorData;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ProximitySensor {
    public ProximitySensor() {
        //Initialize Device
        //The device's default CAN is 0. Change it using AndyMark CAN interface utility
        AM_CAN_Color_Sensor colorSensor = new AM_CAN_Color_Sensor(0);
        //Reset Report Period to the default of 100ms
        colorSensor.resetReportPeriod();

        //UPDATE DEVICE FIRMWARE TO 1.1.1 AND ANDYMARK WPILIB 
        //VERSION TO 2026.0.2 OR NEWER TO TURN LED ON
        //Turn on LED for better data in low light conditions
        colorSensor.turnLedOn();

        //Get data and normalize it using clearC value
        AM_ColorSensorData d = colorSensor.getData();
        double red = (double) d.red / d.clearC;
        double blue = (double) d.blue / d.clearC;
        double green = (double) d.green / d.clearC;
        //Put data on the smart dashboard
        SmartDashboard.putNumber("Color/Red", red);
        SmartDashboard.putNumber("Color/Green", green);
        SmartDashboard.putNumber("Color/Blue", blue);
        SmartDashboard.putNumber("Color/Clear", d.clearC);
        SmartDashboard.putNumber("Color/Proximity", d.proximity);
        SmartDashboard.putNumber("Color/Timestamp(ms)", d.millisStamp);
    }
}
