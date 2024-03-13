// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util.visualization

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.DoubleEntry
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.StringEntry
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.wpilibj.util.Color8Bit
import org.littletonrobotics.junction.LogTable

/**
 * Ligament node on a Mechanism2d. A ligament can have its length changed (like
 * an elevator) or angle changed, like an arm.
 *
 * @see Mechanism2d
 */
class MechanismLigament2d
/**
 * Create a new ligament with the default color (orange) and thickness (6).
 *
 * @param name   The ligament's name.
 * @param input_length The ligament's length.
 * @param input_angle  The ligament's angle relative to its parent in degrees.
 */ @JvmOverloads constructor(
    name: String?,
    /**
     * Set the ligament's length.
     *
     * @param input_length the line length
     */
    /**
     * Get the ligament length.
     *
     * @return the line length
     */
    @set:Synchronized
    @get:Synchronized var input_length: Double,
    /**
     * Set the ligament's angle relative to its parent.
     *
     * @param degrees the angle in degrees
     */
    /**
     * Get the ligament's angle relative to its parent.
     *
     * @return the angle in degrees
     */
    @set:Synchronized
    @get:Synchronized var input_angle: Double,
    /**
     * Set the line thickness.
     *
     * @param weight the line thickness
     */
    /**
     * Get the line thickness.
     *
     * @return the line thickness
     */
    @set:Synchronized
    @get:Synchronized var lineWeight: Double = 10.0,
    /**
     * Set the ligament color.
     *
     * @param color the color of the line
     */
    /**
     * Get the ligament color.
     *
     * @return the color of the line
     */
    @set:Synchronized
    @get:Synchronized var color: Color8Bit = Color8Bit(235, 137, 52)
) :
    MechanismObject2d(name!!) {
    private var m_typePub: StringPublisher? = null
    var angle = input_angle
        set(value) {
            m_angleEntry?.set(value)
            field = value
        }
    private var m_angleEntry: DoubleEntry? = null
    private var m_color: String? = color.toHexString()
    private var m_colorEntry: StringEntry? = null
    var length = input_length
        set(value) {
            m_lengthEntry?.set(value)
            field = value
        }
    private var m_lengthEntry: DoubleEntry? = null
    private var m_weight = lineWeight
    private var m_weightEntry: DoubleEntry? = null

    /**
     * Create a new ligament.
     *
     * @param name      The ligament name.
     * @param length    The ligament length.
     * @param angle     The ligament angle in degrees.
     * @param lineWeight The ligament's line width.
     * @param color     The ligament's color.
     */
    override fun close() {
        super.close()
        m_typePub?.close()
        m_angleEntry?.close()
        m_colorEntry?.close()
        m_lengthEntry?.close()
        m_weightEntry?.close()
    }

    /**
     * Set the ligament's angle relative to its parent.
     *
     * @param angle the angle
     */
    @Synchronized
    fun setAngle(angle: Rotation2d) {
        this.input_angle = angle.degrees
    }

    override fun updateEntries(table: NetworkTable?) {
        m_typePub?.close()
        m_typePub = table?.getStringTopic(".type")?.publish()
        m_typePub?.set("line")

        m_angleEntry?.close()
        m_angleEntry = table?.getDoubleTopic("angle")?.getEntry(0.0)
        m_angleEntry?.set(angle)

        m_lengthEntry?.close()
        m_lengthEntry = table?.getDoubleTopic("length")?.getEntry(0.0)
        m_lengthEntry?.set(length)

        m_colorEntry?.close()
        m_colorEntry = table?.getStringTopic("color")?.getEntry("")
        print(m_color)
        m_colorEntry?.set(m_color)

        m_weightEntry?.close()
        m_weightEntry = table?.getDoubleTopic("weight")?.getEntry(0.0)
        m_weightEntry?.set(m_weight)
    }

    @Synchronized
    override fun akitLog(table: LogTable) {
        table.put(".type", "line")
        table.put("angle", angle)
        table.put("length", length)
        table.put("color", m_color)
        table.put("weight", m_weight)
        super.akitLog(table)
    }
}

