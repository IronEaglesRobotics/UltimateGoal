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

/*
 * Created by Scott Barnes
 *
 * Driver for Amperka's Octoliner sensor. (https://my.amperka.com/modules/octoliner)
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
@I2cDeviceType
@DeviceProperties(name = "Octoliner", description = "Array of infrared sensors from Amperka", xmlTag = "Octoliner")
public class Octoliner extends I2cDeviceSynchDevice<I2cDeviceSynch> implements HardwareDevice {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x2A);
    private boolean debug;
    private double digitalReadThreshold = 0.5;

    private static final short HIGH = 1;
    private static final short LOW = 0;
    private static final short IR_LEDS_PIN = 9;
    private static final int[] SENSOR_PIN_MAP = { 4, 5, 6, 8, 7, 3, 2, 1 };

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    /** Returns the current value of all sensors as an array.
     * @return An array of doubles of length 8. Each element in the array represents the value of
     * the corresponding sensor on the Octoliner.
     */
    public double[] analogRead() {
        double[] all = new double[8];
        for (int i = 0; i < 8; i++) {
            all[i] = analogReadPin(i);
        }
        return all;
    }

    /** Like {@link #analogRead()}, returns the current value of all sensors as an array. With this
     * method, each value is coerced to either a 0 or a 1 based on {@link #digitalReadThreshold} as the threshold.
     * @return An array of integers of length 8. Each element in the array represents the HIGH or LOW state of
     * the corresponding sensor on the Octoliner.
     */
    public int[] digitalRead() {
        int[] all = new int[8];
        for (int i = 0; i < 8; i++) {
            all[i] = digitalReadPin(i);
        }
        return all;
    }

    /** Returns the result of {@link #analogRead()} as a string of comma-delimited values between 0.0 and 1.0.
     * @return A comma-delimited string of the values returned by {@link #analogRead()}.
     */
    @SuppressLint("DefaultLocale")
    public String analogReadString() {
        double[] all = analogRead();
        return String.format("[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", all[0], all[1], all[2], all[3], all[4], all[5], all[6], all[7]);
    }

    /** Returns the result of {@link #digitalRead()} as a string of comma-delimited 1s and 0s.
     *  The string is 8 characters long. A 1 at index 0 indicates that sensor 0 is HIGH and so on.
     * @return A string whose length is exactly 8 characters long consisting of 1s and 0s.
     */
    @SuppressLint("DefaultLocale")
    public String digitalReadString() {
        int[] all = digitalRead();
        return String.format("[%d, %d, %d, %d, %d, %d, %d, %d]", all[0], all[1], all[2], all[3], all[4], all[5], all[6], all[7]);
    }

    /** Gets the result of {@link #digitalRead()} as a bitmap.
     * @return A byte whose LSB is sensor 7 and MSB is sensor 0. If you were to take the result of
     * {@link #digitalReadString()} and interpret it as a byte, you would get this number. 
     */
    public byte digitalReadByte() {
        int[] all = digitalRead();
        return (byte)((all[0] << 7) | (all[1] << 6) | (all[2] << 5) | (all[3] << 4) | (all[4] << 3) | (all[5] << 2) | (all[6] << 1) | (all[7]));
    }

    /** Returns the sum of all digital sensor values.
     * @return A total sum between the value 0 and 8.
     */
    public int digitalSum() {
        int sum = 0;
        int[] all = digitalRead();
        for (int value : all) {
            sum += value;
        }

        return sum;
    }

    /** Return the sum of all analog sensor values.
     * @return A total sum between the values 0.0 and 8.0.
     */
    public double analogSum() {
        double sum = 0;
        double[] all = analogRead();
        for (double v : all) {
            sum += v;
        }

        return sum;
    }

    /** Sets the threshold used by the {@link #digitalRead()} method to distinguish a LOW (0) from a HIGH (1).
     * @param threshold A value between 0.0 and 1.0 at which a sensor should be considered HIGH (1).
     */
    public void setDigitalReadThreshold(double threshold) {
        this.digitalReadThreshold = threshold;
    }

    /** Enables or disables debug logging to RobotLog.
     * @param debug Whether debug logging should be enabled.
     */
    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Octoliner";
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected void writeRegByte(final Register reg, byte value)
    {
        debug("writeByte" + " " + reg.name(), new byte[] { value });
        deviceClient.write8(reg.bVal, value);
    }

    protected void writeRegShort(final Register reg, short value)
    {
        debug("writeShort" + " " + reg.name(), TypeConversion.shortToByteArray(value));
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readRegShort(Register reg)
    {
        short read = TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
        debug("readShort" + " " + reg.name(), TypeConversion.shortToByteArray(read));
        return read;
    }

    private void digitalWritePin(short pin, short value) {
        short data = Short.reverseBytes((short)(0x0001 << pin));
        if (value == HIGH) {
            this.writeRegShort(Register.DIGITAL_WRITE_HIGH, data);
        } else if (value == LOW) {
            this.writeRegShort(Register.DIGITAL_WRITE_LOW, data);
        }
    }

    private double analogReadPin(int pin) {
        short shortPin = (short) SENSOR_PIN_MAP[pin];
        this.writeRegByte(Register.ANALOG_READ, (byte)pin);
        return this.readRegShort(Register.ANALOG_READ) / 4095.0;
    }

    private int digitalReadPin(int pin) {
        return analogReadPin(pin) > digitalReadThreshold ? 1 : 0;
    }

    private void analogWritePin(short pin, double value) {
        value = (int)(value * 255);
        byte[] data = new byte[] { (byte)pin, (byte)value };
        debug("analogWrite", data);
        this.deviceClient.write(Register.ANALOG_WRITE.bVal, data);
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

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Private Utility
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private static String byteToBinaryString(byte n) {
        StringBuilder sb = new StringBuilder("00000000");
        for (int bit = 0; bit < 8; bit++) {
            if (((n >> bit) & 1) > 0) {
                sb.setCharAt(7 - bit, '1');
            }
        }
        return sb.toString();
    }

    private void debug(String context, byte[] data) {
        if (!this.debug) {
            return;
        }

        StringBuilder dataStr = new StringBuilder();
        for (byte datum : data) {
            dataStr.append(byteToBinaryString(datum)).append(" ");
        }
        RobotLog.a(String.format("OCTOLINER (%s): %s", context, dataStr.toString()));
    }

    private void debug(String context, byte data) {
        if (!this.debug) {
            return;
        }

        String dataStr = byteToBinaryString(data);
        RobotLog.a(String.format("OCTOLINER (%s): %s", context, dataStr));
    }

    private void pwmFreq(int frequency) {
        this.writeRegShort(Register.PWM_FREQ, (short)frequency);
    }

    private void pinMode(short pin, Mode mode) {
        short data = Short.reverseBytes((short)(0x0001 << pin));
        switch(mode) {
            case INPUT:
                this.writeRegShort(Register.PORT_MODE_INPUT, data);
                break;
            case INPUT_PULLUP:
                this.writeRegShort(Register.PORT_MODE_PULLUP, data);
                break;
            case INPUT_PULLDOWN:
                this.writeRegShort(Register.PORT_MODE_PULLDOWN, data);
                break;
            case OUTPUT:
                this.writeRegShort(Register.PORT_MODE_OUTPUT, data);
                break;
        }
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
        this.pinMode(IR_LEDS_PIN, Mode.OUTPUT);
        this.digitalWritePin(IR_LEDS_PIN, HIGH);

        this.analogWritePin((byte)0, 0.8);

        this.pwmFreq(8000);

        RobotLog.a(this.deviceClient.getHealthStatus().toString());
        return true;
    }
}
