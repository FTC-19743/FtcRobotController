package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;
import java.util.Arrays;

import java.nio.ByteOrder;

@I2cDeviceType
@DeviceProperties(name = "PixyCam2", description = "PixyCam2.1 on the I2C Bus", xmlTag = "PIXYCAM2")
public class PixyCam2 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x18); // TODO: Fix Number AND This could be a parameter

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods for use in Op Modes
    //
    // Note: To get an instance of this class, use the FTC hardwareMap as follows:
    //       PixyCam2 pixyCam = hardwareMap.get(PixyCam2.class, "pixycam-name");
    //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    public short getHWVersion() { return HWVersion; }
    public short getFWVersion() { return FWVersion; }
    public short getBuildNum() { return buildNum; }
    public String getFWType () { return FWType; };

    public boolean getVersionInfo() {
        if (!versionInfoRetrieved) {
            sendSimpleRequest(VERSION_INFO_REQUEST); // PixyCam should be ready to read results from in < 100 microseconds!
            return readVersionInfo();
        }
        return true;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Internal Helper Methods
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    private boolean logDetails = true;
    private static final byte VERSION_INFO_REQUEST = 0x0e; // Request ID for Version Information from PixyCam
    private static final byte VERSION_INFO_RESPONSE = 0x0f; // Response ID for Version Information from PixyCam
    private  static final int HEADER_LENGTH = 4;

    private boolean versionInfoRetrieved = false;
    private short HWVersion, FWVersion, buildNum;
    private String FWType = "";





    private boolean readVersionInfo() {
        if (logDetails) teamUtil.log("PixyCam: Reading Version Info");

        byte[] rawVersionInfo = readResponse(VERSION_INFO_RESPONSE);
        if (logDetails) teamUtil.log("PixyCam: Returned Data: "+ Arrays.toString(rawVersionInfo));

        if (rawVersionInfo == null || rawVersionInfo.length != 16) {
            // failed to get data back
            if (logDetails) teamUtil.log("PixyCam: Bad Version Data");
            return false;
        } else {
            versionInfoRetrieved = true;
            HWVersion = TypeConversion.byteArrayToShort(rawVersionInfo,0, ByteOrder.LITTLE_ENDIAN);
            FWVersion = TypeConversion.byteArrayToShort(rawVersionInfo,2, ByteOrder.LITTLE_ENDIAN);
            buildNum = TypeConversion.byteArrayToShort(rawVersionInfo,4, ByteOrder.LITTLE_ENDIAN);
            FWType = "Not Implemented"; // TODO: convert the rest of the byte array into the string
            return true;
        }

    }

    // Send a simple 4 byte request to PixyCam with the specified request type code
    private void sendSimpleRequest (byte type) {
        byte[] request = {  (byte)0xae,  // first two bytes specify no check-sum data
                (byte)0xc1,
                type,  // the code for the request type
                (byte)0x00}; // no extra data included in this request
        if (logDetails) teamUtil.log("PixyCam: Send Request:"+ Arrays.toString(request));

        deviceClient.write(request);

    }

    // Read a response from PixyCam, validate the response type and return the payload data
    private byte[] readResponse(byte expectedResponseType) {
        if (logDetails) teamUtil.log("PixyCam: Read Response");

        byte[] headerResult = deviceClient.read(HEADER_LENGTH);
        if (logDetails) teamUtil.log("PixyCam: Read Response:"+ Arrays.toString(headerResult));

        if (headerResult.length < 4 ) {
            // error: didn't get at least 4 bytes back
            if (logDetails) teamUtil.log("PixyCam: Read Response < 4 bytes");
            return null;
        } else if (headerResult[2] != expectedResponseType) {
            // error: unexpected response type
            if (logDetails) teamUtil.log("PixyCam: Response Code Mismatch:" + expectedResponseType +":" + headerResult[2]);
            return null;
        }
        if (headerResult[0] == (byte) 0xaf) { // checksum data included
            byte[] checkSumResult = deviceClient.read(2); // read the first two bytes for check sum
            if (logDetails) teamUtil.log("PixyCam: Response Checksum:" + TypeConversion.byteArrayToShort(checkSumResult, ByteOrder.LITTLE_ENDIAN));
            // TODO: Validate checksum?
        }
        return deviceClient.read(headerResult[3]); // read and return the data payload
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods called by the FTC SDK
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    public PixyCam2(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged

        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "PixyCam 2.1";
    }
}
