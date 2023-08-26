package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Map;

public class DiscoverHardware {

    public static class DeviceInfo {
        public String deviceName;
        public String deviceType;
        public HardwareDevice device;

        public DeviceInfo(String deviceName, HardwareDevice device) {
            this.deviceName = device.getDeviceName();
            this.deviceType = device.getClass().getName();
            this.device = device;
        }
    }

    private final Collection<DeviceInfo> devices;

    private static <DEVICE_TYPE extends HardwareDevice> Collection<DeviceInfo> getDevices(HardwareMap.DeviceMapping<DEVICE_TYPE> deviceMapping) {
        Collection<DeviceInfo> devices = new ArrayList<>();

        for (Map.Entry<String, DEVICE_TYPE> entry : deviceMapping.entrySet()) {
            DeviceInfo device = new DeviceInfo(entry.getKey(), entry.getValue());
            devices.add(device);
        }

        return devices;
    }

    public DiscoverHardware(HardwareMap hwMap) {
        devices = new ArrayList<>();

        Iterator<HardwareDevice> it = hwMap.iterator();
        while (it.hasNext()) {
            HardwareDevice hd = it.next();
            DeviceInfo di = new DeviceInfo(hd.getDeviceName(), hd);
            // DeviceInfo di = new DeviceInfo(hd.getClass().getName(), hd);
            devices.add(di);

            // "Expansion Hub Voltage Sensor"
            // com.qualcomm.hardware.lynx.LynxVoltageSensor vs;
            // vs.getVoltage();

            // com.qualcomm.hardware.lynx.LynxModule lm;
            // lm.getAuxiliaryVoltage();
            // lm.getInputVoltage(<value>);

            // expansion hub: com.qualcomm.hardware.lynx.LynxUsbDeviceDelegate
            // Expansion Hub Servo Controller: com.qualcomm.hardware.lynx.LynxServoController
            // Expansion Hub Voltage Sensor: com.qualcomm.hardware.lynx.LynxVoltageSensor
            // Expansion Hub: com.qualcomm.hardware.lynx.LynxModule
        }
        /*
        devices.addAll(getDevices(hwMap.accelerationSensor));
        devices.addAll(getDevices(hwMap.analogInput));
        devices.addAll(getDevices(hwMap.colorSensor));
        devices.addAll(getDevices(hwMap.compassSensor));
        devices.addAll(getDevices(hwMap.crservo));
        devices.addAll(getDevices(hwMap.dcMotor));
        devices.addAll(getDevices(hwMap.dcMotorController));
        devices.addAll(getDevices(hwMap.gyroSensor));
        devices.addAll(getDevices(hwMap.i2cDevice));
        devices.addAll(getDevices(hwMap.i2cDeviceSynch));
        devices.addAll(getDevices(hwMap.irSeekerSensor));
        devices.addAll(getDevices(hwMap.led));
        devices.addAll(getDevices(hwMap.lightSensor));
        devices.addAll(getDevices(hwMap.opticalDistanceSensor));
        devices.addAll(getDevices(hwMap.pwmOutput));
        devices.addAll(getDevices(hwMap.servo));
        devices.addAll(getDevices(hwMap.servoController));
        devices.addAll(getDevices(hwMap.touchSensor));
        devices.addAll(getDevices(hwMap.touchSensorMultiplexer));
        devices.addAll(getDevices(hwMap.ultrasonicSensor));
        devices.addAll(getDevices(hwMap.voltageSensor));
        */

        /*
        Can look into adding other hardware devices, by getting all instances. This can work for
        IMUs and the like.
         */
    }

    public Collection<DeviceInfo> getDevices() {
        return devices;
    }
}
