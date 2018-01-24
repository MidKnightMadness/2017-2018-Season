/*package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

/**
 * Created by gregory.ling on 1/23/18.
 *\/

@I2cSensor(xmlTag = "DRIVER", name = "Driver", description = "Driver Controls: Normal (most people) or Inverted (Alex)")
public class Driver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    Driver(I2cDeviceSynch deviceSynch) {
        super(deviceSynch, true);
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Normal";
    }
}*/
