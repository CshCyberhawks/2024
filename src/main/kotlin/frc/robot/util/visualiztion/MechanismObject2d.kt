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

import edu.wpi.first.networktables.NetworkTable
import org.littletonrobotics.junction.LogTable

/**
 * Common base class for all Mechanism2d node types.
 *
 *
 *
 * To append another node, call [.append]. Objects that
 * aren't appended to a published [Mechanism2d] container are
 * nonfunctional.
 *
 * @see Mechanism2d
 */
abstract class MechanismObject2d
/**
 * Create a new Mechanism node object.
 *
 * @param name the node's name, must be unique.
 */ protected constructor(
    /** Relative to parent.  */
    val name: String
) : AutoCloseable {
    private var m_table: NetworkTable? = null
    private val m_objects: MutableMap<String, MechanismObject2d> = HashMap(1)
    override fun close() {
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

    @Synchronized
    fun update(table: NetworkTable?) {
        m_table = table
        updateEntries(m_table)
        for (obj in m_objects.values) {
            obj.update(m_table!!.getSubTable(obj.name))
        }
    }

    /**
     * Update all entries with new ones from a new table.
     *
     * @param table the new table.
     */
    protected abstract fun updateEntries(table: NetworkTable?)

    @Synchronized
    open fun akitLog(table: LogTable) {
        for (obj in m_objects.values) {
            obj.akitLog(table.getSubtable(obj.name))
        }
    }
}

