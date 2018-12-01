package frc.team9761.robot.Drivers

// import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.GyroBase
import edu.wpi.first.wpilibj.PIDSource
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.interfaces.Gyro
import edu.wpi.first.wpilibj.livewindow.LiveWindow

import java.nio.ByteBuffer
import java.nio.ByteOrder

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2015-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/* MODIFIED BY TEAM 254                                                       */
/*----------------------------------------------------------------------------*/

/**
 * Use a rate gyro to return the robots heading relative to a starting position. The Gyro class tracks the robots
 * heading based on the starting position. As the robot rotates the new heading is computed by integrating the rate of
 * rotation returned by the sensor. When the class is instantiated, it does a short calibration routine where it samples
 * the gyro while at rest to determine the default offset. This is subtracted from each sample to determine the heading.

 * This class is for the digital ADXRS453 gyro sensor that connects via SPI. A datasheet can be found here:
 * http://www.analog.com/media/en/technical-documentation/data-sheets/ADXRS453. pdf
 */
class ADXRS453_Gyro
/**
 * Constructor.

 * @param port
 * *            (the SPI port that the gyro is connected to)
 */
@JvmOverloads constructor(port: SPI.Port = SPI.Port.kOnboardCS0) : GyroBase(), Gyro, PIDSource {

    private var m_spi: SPI? = null

    private var m_is_calibrating: Boolean = false
    @get:Synchronized var center: Double = 0.toDouble()
        private set

    init {
        m_spi = SPI(port)
        m_spi!!.setClockRate(3000000)
        m_spi!!.setMSBFirst()
        m_spi!!.setSampleDataOnRising()
        m_spi!!.setClockActiveHigh()
        m_spi!!.setChipSelectActiveLow()

        /** Validate the part ID  */
        if (readRegister(kPIDRegister) and 0xff00 != 0x5200) {
            m_spi!!.free()
            m_spi = null
            DriverStation.reportError("Could not find ADXRS453 gyro on SPI port " + port.value, false)
        } else {

            m_spi!!.initAccumulator(kSamplePeriod, 0x20000000, 4, 0x0c00000E, 0x04000000, 10, 16, true, true)

            calibrate()

            LiveWindow.addSensor("ADXRS453_Gyro", port.value, this)
        }
    }

    /**
     * This is a blocking calibration call. There are also non-blocking options available in this class!

     * {@inheritDoc}
     */
    @Synchronized override fun calibrate() {
        Timer.delay(0.1)
        startCalibrate()
        Timer.delay(kCalibrationSampleTime)
        endCalibrate()
    }

    @Synchronized fun startCalibrate() {
        if (m_spi == null)
            return

        if (!m_is_calibrating) {
            m_is_calibrating = true
            m_spi!!.setAccumulatorCenter(0)
            m_spi!!.resetAccumulator()
        }
    }

    @Synchronized fun endCalibrate() {
        if (m_is_calibrating) {
            m_is_calibrating = false
            center = m_spi!!.accumulatorAverage
            m_spi!!.setAccumulatorCenter(Math.round(center).toInt())
            m_spi!!.resetAccumulator()
        }
    }

    @Synchronized fun cancelCalibrate() {
        if (m_is_calibrating) {
            m_is_calibrating = false
            m_spi!!.setAccumulatorCenter(Math.round(center).toInt())
            m_spi!!.resetAccumulator()
        }
    }

    private fun calcParity(v: Int): Boolean {
        var v = v
        var parity = false
        while (v != 0) {
            parity = !parity
            v = v and v - 1
        }
        return parity
    }

    private fun readRegister(reg: Int): Int {
        val cmdhi = 0x8000 or (reg shl 1)
        val parity = calcParity(cmdhi)

        val buf = ByteBuffer.allocateDirect(4)
        buf.order(ByteOrder.BIG_ENDIAN)
        buf.put(0, (cmdhi shr 8).toByte())
        buf.put(1, (cmdhi and 0xff).toByte())
        buf.put(2, 0.toByte())
        buf.put(3, (if (parity) 0 else 1).toByte())

        m_spi!!.write(buf, 4)
        m_spi!!.read(false, buf, 4)

        if (buf.get(0).toInt() and 0xe0 == 0) {
            return 0
        }
        return buf.getInt(0) shr 5 and 0xffff
    }

    /**
     * {@inheritDoc}
     */
    @Synchronized override fun reset() {
        if (m_is_calibrating) {
            cancelCalibrate()
        }
        m_spi!!.resetAccumulator()
    }

    /**
     * Delete (free) the spi port used for the gyro and stop accumulating.
     */
    override fun free() {
        if (m_spi != null) {
            m_spi!!.free()
            m_spi = null
        }
    }

    /**
     * {@inheritDoc}
     */
    @Synchronized override fun getAngle(): Double {
        if (m_spi == null)
            return 0.0
        if (m_is_calibrating) {
            return 0.0
        }
        return m_spi!!.accumulatorValue.toDouble() * kDegreePerSecondPerLSB * kSamplePeriod
    }

    /**
     * {@inheritDoc}
     */
    @Synchronized override fun getRate(): Double {
        if (m_spi == null)
            return 0.0
        if (m_is_calibrating) {
            return 0.0
        }
        return m_spi!!.accumulatorLastValue * kDegreePerSecondPerLSB
    }

    companion object {
        val kCalibrationSampleTime = 5.0

        private val kSamplePeriod = 0.001
        private val kDegreePerSecondPerLSB = -0.0125

        private val kPIDRegister = 0x0C
    }
}
/**
 * Constructor. Uses the onboard CS0.
 */