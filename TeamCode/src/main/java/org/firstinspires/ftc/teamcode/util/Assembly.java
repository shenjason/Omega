package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Assembly - Abstract base class for all robot subsystems (Robot, Shooter, Turret).
 *
 * Provides a common framework for hardware initialization, periodic updates,
 * alliance side tracking, and conditional debug telemetry output.
 *
 * Subclasses must implement:
 *   - hardwareInit() — called in the constructor to map and configure hardware devices
 *   - update()       — called every loop cycle to run the subsystem's control logic
 */
public abstract class Assembly {

    /** Alliance side constants used throughout the codebase */
    public static final boolean SIDE_RED = false;
    public static final boolean SIDE_BLUE = true;

    protected Telemetry t;
    protected HardwareMap hardwareMap;
    /** Whether debug telemetry is enabled and which alliance side the robot is on */
    protected boolean debug, side = false;


    /**
     * Initializes the assembly and automatically calls hardwareInit().
     * @param _hardwareMap FTC hardware map for device access
     * @param _t           Telemetry instance for debug output
     * @param _debug       Enable/disable debug telemetry
     * @param _side        Alliance side (SIDE_BLUE = true, SIDE_RED = false)
     */
    public Assembly(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side){
        t = _t;
        hardwareMap = _hardwareMap;
        debug = _debug;
        side = _side;

        hardwareInit(); // Subclass-specific hardware setup
    }

    /** Maps and configures hardware devices. Called once during construction. */
    public abstract void hardwareInit();

    /** Runs the subsystem's control logic. Called every loop cycle from the OpMode. */
    public abstract void update();

    /** Adds a telemetry line header (only if debug mode is enabled) */
    protected void debugAddLine(String line){
        if (!debug) return;
        t.addLine(line);
    }

    /** Adds a telemetry key-value data pair (only if debug mode is enabled) */
    protected void debugAddData(String header, Object data){
        if (!debug) return;
        t.addData(header, data);
    }

}
