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

import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.util.Color8Bit
import org.littletonrobotics.junction.LogTable

/**
 * Visual 2D representation of arms, elevators, and general mechanisms through a
 * node-based API.
 *
 *
 *
 * A Mechanism2d object is published and contains at least one root node. A root
 * is the anchor point of other nodes (such as ligaments). Other nodes are
 * recursively appended based on other nodes.
 *
 * @see MechanismObject2d
 *
 * @see MechanismLigament2d
 *
 * @see MechanismRoot2d
 */
class Mechanism2d @JvmOverloads constructor(
    width: Double,
    height: Double,
    backgroundColor: Color8Bit = Color8Bit(0, 0, 32)
) :
    NTSendable, AutoCloseable {
    private var m_table: NetworkTable? = null
    private val m_roots: MutableMap<String, MechanismRoot2d>

    //    private val m_ligaments: MutableMap<String, MechanismLigament2d>
    private val m_dims = DoubleArray(2)
    private var m_color: String? = null
    private var m_dimsPub: DoubleArrayPublisher? = null
    private var m_colorPub: StringPublisher? = null
    /**
     * Create a new Mechanism2d with the given dimensions.
     *
     *
     *
     * The dimensions represent the canvas that all the nodes are drawn on.
     *
     * @param width           the width
     * @param height          the height
     * @param backgroundColor the background color. Defaults to dark blue.
     */
    /**
     * Create a new Mechanism2d with the given dimensions and default color (dark
     * blue).
     *
     *
     *
     * The dimensions represent the canvas that all the nodes are drawn on.
     *
     * @param width  the width
     * @param height the height
     */
    init {
        m_roots = HashMap()
//        m_ligaments = HashMap()
        m_dims[0] = width
        m_dims[1] = height
        setBackgroundColor(backgroundColor)
    }

    override fun close() {
        m_dimsPub?.close()
        m_colorPub?.close()
        for (root in m_roots.values) {
            root.close()
        }
    }

    /**
     * Get or create a root in this Mechanism2d with the given name and position.
     *
     *
     *
     * If a root with the given name already exists, the given x and y coordinates
     * are not used.
     *
     * @param name the root name
     * @param x    the root x coordinate
     * @param y    the root y coordinate
     * @return a new root joint object, or the existing one with the given name.
     */
    @Synchronized
    fun getRoot(name: String, x: Double, y: Double): MechanismRoot2d {
        val existing = m_roots[name]
        if (existing != null) {
            return existing
        }
        val root = MechanismRoot2d(name, x, y)
        m_roots[name] = root
        root.update(m_table?.getSubTable(name))
        return root
    }

//    fun getLigament(name: String, length: Double, angle: Double): MechanismLigament2d {
//        val existing = m_ligaments[name]
//        if (existing != null) {
//            return existing
//        }
//        val root = MechanismLigament2d(name, length, angle)
//        m_ligaments[name] = root
//        root.update(m_table?.getSubTable(name))
//        return root
//    }

    /**
     * Set the Mechanism2d background color.
     *
     * @param color the new color
     */
    @Synchronized
    fun setBackgroundColor(color: Color8Bit) {
        m_color = color.toHexString()
        m_colorPub?.set(m_color)
    }

    override fun initSendable(builder: NTSendableBuilder) {
        builder.setSmartDashboardType("Mechanism2d")
        synchronized(this) {
            m_table = builder.table
            m_dimsPub?.close()
            m_dimsPub = m_table?.getDoubleArrayTopic("dims")?.publish()
            m_dimsPub?.set(m_dims)
            m_colorPub?.close()
            m_colorPub = m_table?.getStringTopic("backgroundColor")?.publish()
            m_colorPub?.set(m_color)
            for ((name, root) in m_roots) {
                synchronized(root) { root.update(m_table?.getSubTable(name)) }
            }
//            for ((name, ligament) in m_ligaments) {
//                synchronized(ligament) { ligament.update(m_table?.getSubTable(name)) }
//            }
        }
    }

    @Synchronized
    fun akitLog(table: LogTable) {
        table.put(".type", "Mechanism2d")
        table.put(".controllable", false)
        table.put("dims", m_dims)
        table.put("backgroundColor", m_color)
        for ((name, root) in m_roots) {
            synchronized(root) { root.akitLog(table.getSubtable(name)) }
        }
    }
}

