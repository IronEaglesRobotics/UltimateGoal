package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Type;
import java.math.BigInteger;
import java.nio.ByteOrder;

@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
@I2cDeviceType
@DeviceProperties(name = "Octoliner", description = "Array of infrared sensors from Amperka", xmlTag = "Octoliner")
public class Octoliner extends I2cDeviceSynchDevice<I2cDeviceSynch> implements HardwareDevice {
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x2A);
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Raw Register Reads
    ////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected void writeByte(final Register reg, byte value)
    {
        logByteArray("writeByte" + " " + reg.name(), new byte[] { value });
        deviceClient.write8(reg.bVal, value);
    }

    protected void writeShort(final Register reg, short value)
    {
        logByteArray("writeShort" + " " + reg.name(), TypeConversion.shortToByteArray(value));
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        short read = TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
        logByteArray("readShort" + " " + reg.name(), TypeConversion.shortToByteArray(read));
        return read;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public enum Register {
        WHO_AM_I (0x00),
        RESET (0x01),
        CHANGE_I2C_ADDR (0x02),
        SAVE_I2C_ADDR (0x03),
        PORT_MODE_INPUT (0x04),
        PORT_MODE_PULLUP (0x05),
        PORT_MODE_PULLDOWN (0x06),
        PORT_MODE_OUTPUT (0x07),
        DIGITAL_READ (0x08),
        DIGITAL_WRITE_HIGH (0x09),
        DIGITAL_WRITE_LOW (0x0A),
        ANALOG_WRITE (0x0B),
        ANALOG_READ (0x0C),
        PWM_FREQ (0x0D),
        ADC_SPEED (0x0E);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    private enum Mode {
        INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN
    }

    private static final short HIGH = 1;
    private static final short LOW = 0;
    private static final short irLedsPin = 9;
    private static final int[] sensorPinMap = { 4, 5, 6, 8, 7, 3, 2, 1 };

//    private int reverseUnit16(short data) {
//        return ((data & 0xff) << 8) | ((data>>8) & 0xff);
//    }
//
//    private int reverseUnit16(byte[] data) {
//        return TypeConversion.byteArrayToInt(data, ByteOrder.LITTLE_ENDIAN);
//    }

    public static String toBinaryString(byte n) {
        StringBuilder sb = new StringBuilder("00000000");
        for (int bit = 0; bit < 8; bit++) {
            if (((n >> bit) & 1) > 0) {
                sb.setCharAt(7 - bit, '1');
            }
        }
        return sb.toString();
    }
    private void logByteArray(String context, byte[] data) {
        String dataStr = "";
        for (int i = 0; i < data.length; i++) {
            dataStr += toBinaryString(data[i]) + " ";
        }
        RobotLog.a(String.format("OCTOLINER (%s): %s", context, dataStr));
    }

    private void digitalWrite(short pin, short value) {
        short data = Short.reverseBytes((short)(0x0001 << pin));
        if (value == HIGH) {
            this.writeShort(Register.DIGITAL_WRITE_HIGH, data);
        } else if (value == LOW) {
            this.writeShort(Register.DIGITAL_WRITE_LOW, data);
        }
    }

    public short analogRead16(short pin) {
        this.writeByte(Register.ANALOG_READ, (byte)pin);
        return this.readShort(Register.ANALOG_READ);
    }

    public double analogRead(int pin) {
        short shortPin = (short)sensorPinMap[pin];
        return this.analogRead16(shortPin) / 4095.0;
    }

    public int digitalRead(int pin) {
        return analogRead(pin) > 0.5 ? 1 : 0;
    }

    public double[] analogReadAll() {
        double[] all = new double[8];
        for (int i = 0; i < 8; i++) {
            all[i] = analogRead(i);
        }
        return all;
    }

    public int[] digitalReadAll() {
        int[] all = new int[8];
        for (int i = 0; i < 8; i++) {
            all[i] = digitalRead(i);
        }
        return all;
    }

    @SuppressLint("DefaultLocale")
    public String analogReadAllString() {
        double[] all = analogReadAll();
        return String.format("[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", all[0], all[1], all[2], all[3], all[4], all[5], all[6], all[7]);
    }

    @SuppressLint("DefaultLocale")
    public String digitalReadAllString() {
        int[] all = digitalReadAll();
        return String.format("[%d, %d, %d, %d, %d, %d, %d, %d]", all[0], all[1], all[2], all[3], all[4], all[5], all[6], all[7]);
    }

    public int digitalCount() {
        int count = 0;
        int[] all = digitalReadAll();
        for(int i = 0; i < all.length; i++) {
            count += all[i];
        }

        return count;
    }
    private void pwmFreq(int frequency) {
        this.writeShort(Register.PWM_FREQ, (short)frequency);
    }

//    private void changeAddr(I2cAddr address) {
//        this.deviceClient.write(Register.CHANGE_I2C_ADDR.bVal, TypeConversion.intToByteArray(address.get7Bit()));
//    }

//    private void saveAddr() {
//        this.deviceClient.write(Register.SAVE_I2C_ADDR.bVal, null);
//    }
//
//    private void reset() {
//        this.deviceClient.write(Register.RESET.bVal, null);
//    }

    private void pinMode(short pin, Mode mode) {
        short data = Short.reverseBytes((short)(0x0001 << pin));
        switch(mode) {
            case INPUT:
                this.writeShort(Register.PORT_MODE_INPUT, data);
                break;
            case INPUT_PULLUP:
                this.writeShort(Register.PORT_MODE_PULLUP, data);
                break;
            case INPUT_PULLDOWN:
                this.writeShort(Register.PORT_MODE_PULLDOWN, data);
                break;
            case OUTPUT:
                this.writeShort(Register.PORT_MODE_OUTPUT, data);
                break;
        }
    }

    private void analogWrite(short pin, double value) {
        value = (int)(value * 255);
        byte[] data = new byte[] { (byte)pin, (byte)value };
        logByteArray("analogWrite", data);
        this.deviceClient.write(Register.ANALOG_WRITE.bVal, data);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public Octoliner(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public Octoliner(I2cDeviceSynch deviceClient) {
        this(deviceClient, true);
    }

    @Override
    protected synchronized boolean doInitialize() {
        this.pinMode(irLedsPin, Mode.OUTPUT);
        this.digitalWrite(irLedsPin, HIGH);

        this.analogWrite((byte)0, 0.8);

        this.pwmFreq(8000);

        RobotLog.a(this.deviceClient.getHealthStatus().toString());
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Octoliner";
    }
}
