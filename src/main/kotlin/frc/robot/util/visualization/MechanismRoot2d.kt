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

import edu.wpi.first.networktables.DoublePublisher
import edu.wpi.first.networktables.NetworkTable
import org.littletonrobotics.junction.LogTable


/**
 * Root Mechanism2d node.
 *
 *
 *
 * A root is the anchor point of other nodes (such as ligaments).
 *
 *
 *
 * Do not create objects of this class directly! Obtain instances from the
 * [Mechanism2d.getRoot] factory method.
 *
 *
 *
 * Append other nodes by using [.append].
 */
class MechanismRoot2d
/**
 * Package-private constructor for roots.
 *
 * @param name name
 * @param x    x coordinate of root (provide only when constructing a root node)
 * @param y    y coordinate of root (provide only when constructing a root node)
 */ internal constructor(val name: String, private var m_x: Double, private var m_y: Double) : AutoCloseable {
    private var m_table: NetworkTable? = null
    private val m_objects: MutableMap<String, MechanismObject2d> = HashMap(1)
    private var m_xPub: DoublePublisher? = null
    private var m_yPub: DoublePublisher? = null
    override fun close() {
        m_xPub?.close()
        m_yPub?.close()
        for (obj in m_objects.values) {
            obj.close()
        }
    }

    /**
     * Append a Mechanism object that is based on this one.
     *
     * @param <T>    The object type.
     * @param object the object to add.
     * @return the object given as a parameter, useful for variable assignments and
     * call chaining.
     * @throws UnsupportedOperationException if the object's name is already used -
     * object names must
     * be unique.
    </T> */
    @Synchronized
    fun <T : MechanismObject2d?> append(`object`: T): T {
        if (m_objects.containsKey(`object`!!.name)) {
            throw UnsupportedOperationException("Mechanism object names must be unique!")
        }
        m_objects[`object`.name] = `object`
        if (m_table != null) {
            `object`.update(m_table!!.getSubTable(`object`.name))
        }
        return `object`
    }

    /**
     * Set the root's position.
     *
     * @param x new x coordinate
     * @param y new y coordinate
     */
    @Synchronized
    fun setPosition(x: Double, y: Double) {
        m_x = x
        m_y = y
        flush()
    }

    @Synchronized
    fun update(table: NetworkTable?) {
        m_table = table
        m_xPub?.close()
        m_xPub = m_table?.getDoubleTopic("x")?.publish()
        m_yPub?.close()
        m_yPub = m_table?.getDoubleTopic("y")?.publish()
        flush()
        for (obj in m_objects.values) {
            obj.update(m_table!!.getSubTable(obj.name))
        }
    }

    private fun flush() {
        if (m_xPub != null) {
            m_xPub!!.set(m_x)
        }
        if (m_yPub != null) {
            m_yPub!!.set(m_y)
        }
    }

    @Synchronized
    fun akitLog(table: LogTable) {
        table.put("x", m_x)
        table.put("y", m_y)
        for (obj in m_objects.values) {
            obj.akitLog(table.getSubtable(obj.name))
        }
    }
}

