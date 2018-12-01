package frc.team9761.robot.Drivers

import edu.wpi.first.wpilibj.AnalogInput

import java.util.LinkedList

/**
 * Driver for an analog Ultrasonic Sensor (mainly to help smooth out noise).
 */
class UltrasonicSensor(port: Int) {
    protected var mAnalogInput: AnalogInput
    private val cache: LinkedList<Double>
    protected var mScalingFactor = 512.0 / 5.0

    init {
        mAnalogInput = AnalogInput(port)
        cache = LinkedList<Double>()
        cache.add(rawDistance)
    }

    fun update() {
        cache.add(rawDistance)
        if (cache.size > kCacheSize)
            cache.removeFirst()
    }

    val rawDistance: Double
        get() = mAnalogInput.voltage * mScalingFactor

    val averageDistance: Double
        get() {
            var total = 0.0
            for (d in cache) {
                total += d
            }
            return total / cache.size
        }

    val latestDistance: Double
        get() = cache.last

    companion object {

        private val kCacheSize = 5
    }
}