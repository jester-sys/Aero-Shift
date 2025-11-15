package com.exmple.aeroshift.custom


import kotlin.math.*

/**
 * REAL PHYSICS ENGINE for Rocket Simulation
 * Includes: Newtonian mechanics, orbital dynamics, gravity calculations,
 * fuel consumption, thrust, drag, and realistic trajectory calculations
 */
class RocketPhysicsEngine {

    // Physical Constants
    companion object {
        // Gravitational constants
        const val G = 6.674e-11 // Gravitational constant (m³/kg·s²)
        const val EARTH_MASS = 5.972e24 // kg
        const val MOON_MASS = 7.342e22 // kg
        const val EARTH_RADIUS = 6.371e6 // meters
        const val MOON_RADIUS = 1.737e6 // meters
        const val EARTH_MOON_DISTANCE = 3.844e8 // meters (384,400 km)

        // Atmospheric constants
        const val SEA_LEVEL_PRESSURE = 101325.0 // Pa
        const val AIR_DENSITY_SEA_LEVEL = 1.225 // kg/m³
        const val SCALE_HEIGHT = 8500.0 // meters (atmospheric scale height)
        const val ATMOSPHERE_HEIGHT = 100000.0 // meters (Kármán line)

        // Rocket constants
        const val ROCKET_DRY_MASS = 2000.0 // kg (empty rocket)
        const val FUEL_MASS_INITIAL = 3000.0 // kg (initial fuel)
        const val SPECIFIC_IMPULSE = 300.0 // seconds (engine efficiency)
        const val EXHAUST_VELOCITY = SPECIFIC_IMPULSE * 9.81 // m/s
        const val MAX_THRUST = 50000.0 // Newtons
        const val FUEL_CONSUMPTION_RATE = 10.0 // kg/s at full throttle
        const val ROCKET_CROSS_SECTION = 2.0 // m² (drag area)
        const val DRAG_COEFFICIENT = 0.5 // typical rocket

        // Simulation
        const val TIME_STEP = 0.016 // seconds (60 FPS)
        const val MAX_VELOCITY = 15000.0 // m/s (escape velocity consideration)
    }

    /**
     * Rocket State - stores all physical properties
     */
    data class RocketState(
        // Position (relative to Earth center)
        var x: Double = EARTH_RADIUS, // meters from Earth center
        var y: Double = 0.0,

        // Velocity
        var vx: Double = 0.0, // m/s
        var vy: Double = 0.0, // m/s

        // Rotation (angle in radians, 0 = pointing up)
        var angle: Double = PI / 2, // 90 degrees = vertical

        // Mass properties
        var fuelMass: Double = FUEL_MASS_INITIAL,
        var totalMass: Double = ROCKET_DRY_MASS + FUEL_MASS_INITIAL,

        // Engine state
        var throttle: Double = 0.0, // 0.0 to 1.0
        var isEngineOn: Boolean = false,

        // Computed properties
        var altitude: Double = 0.0, // meters above surface
        var velocity: Double = 0.0, // m/s (magnitude)
        var acceleration: Double = 0.0, // m/s²
        var gravity: Double = 9.81, // m/s² (local gravity)

        // Mission data
        var missionTime: Double = 0.0, // seconds
        var distanceToMoon: Double = EARTH_MOON_DISTANCE,
        var inOrbit: Boolean = false,
        var hasLanded: Boolean = false
    )

    private var state = RocketState()

    /**
     * Initialize rocket at launch pad
     */
    fun initialize() {
        state = RocketState(
            x = EARTH_RADIUS, // At Earth surface
            y = 0.0,
            vx = 0.0,
            vy = 0.0,
            angle = PI / 2, // Pointing up
            fuelMass = FUEL_MASS_INITIAL,
            totalMass = ROCKET_DRY_MASS + FUEL_MASS_INITIAL,
            throttle = 0.0,
            isEngineOn = false
        )
        updateDerivedProperties()
    }

    /**
     * Main physics update - called every frame
     * Returns updated state
     */
    fun update(deltaTime: Double = TIME_STEP): RocketState {
        if (state.hasLanded) return state

        // 1. Calculate forces
        val forces = calculateForces()

        // 2. Apply Newton's second law: F = ma → a = F/m
        val ax = forces.fx / state.totalMass
        val ay = forces.fy / state.totalMass

        // 3. Update velocity (v = v0 + at)
        state.vx += ax * deltaTime
        state.vy += ay * deltaTime

        // Limit max velocity (for numerical stability)
        val speed = sqrt(state.vx * state.vx + state.vy * state.vy)
        if (speed > MAX_VELOCITY) {
            val factor = MAX_VELOCITY / speed
            state.vx *= factor
            state.vy *= factor
        }

        // 4. Update position (x = x0 + vt)
        state.x += state.vx * deltaTime
        state.y += state.vy * deltaTime

        // 5. Consume fuel if engine is on
        if (state.isEngineOn && state.fuelMass > 0) {
            val fuelConsumed = FUEL_CONSUMPTION_RATE * state.throttle * deltaTime
            state.fuelMass = max(0.0, state.fuelMass - fuelConsumed)
            state.totalMass = ROCKET_DRY_MASS + state.fuelMass

            // Engine cuts off when fuel depleted
            if (state.fuelMass <= 0) {
                state.isEngineOn = false
                state.throttle = 0.0
            }
        }

        // 6. Update time
        state.missionTime += deltaTime

        // 7. Update derived properties
        updateDerivedProperties()

        // 8. Check landing condition
        checkLanding()

        return state
    }

    /**
     * Calculate all forces acting on the rocket
     */
    private fun calculateForces(): Forces {
        var fx = 0.0
        var fy = 0.0

        // 1. GRAVITY FORCE (Earth)
        val earthGravity = calculateGravity(
            state.x, state.y,
            0.0, 0.0, // Earth at origin
            EARTH_MASS
        )
        fx += earthGravity.fx
        fy += earthGravity.fy

        // 2. GRAVITY FORCE (Moon)
        val moonX = EARTH_MOON_DISTANCE // Moon to the right
        val moonY = 0.0
        val moonGravity = calculateGravity(
            state.x, state.y,
            moonX, moonY,
            MOON_MASS
        )
        fx += moonGravity.fx
        fy += moonGravity.fy

        // 3. THRUST FORCE
        if (state.isEngineOn && state.fuelMass > 0) {
            val thrust = MAX_THRUST * state.throttle
            fx += thrust * cos(state.angle)
            fy += thrust * sin(state.angle)
        }

        // 4. ATMOSPHERIC DRAG (only in atmosphere)
        if (state.altitude < ATMOSPHERE_HEIGHT) {
            val drag = calculateDrag()
            fx += drag.fx
            fy += drag.fy
        }

        return Forces(fx, fy)
    }

    /**
     * Calculate gravitational force from a celestial body
     * Using Newton's law of universal gravitation: F = G*m1*m2/r²
     */
    private fun calculateGravity(
        x1: Double, y1: Double, // Rocket position
        x2: Double, y2: Double, // Body position
        bodyMass: Double
    ): Forces {
        val dx = x2 - x1
        val dy = y2 - y1
        val distanceSquared = dx * dx + dy * dy
        val distance = sqrt(distanceSquared)

        // Avoid division by zero
        if (distance < 1.0) return Forces(0.0, 0.0)

        // Gravitational force magnitude: F = G*m1*m2/r²
        val forceMagnitude = (G * state.totalMass * bodyMass) / distanceSquared

        // Force components (pointing toward body)
        val fx = forceMagnitude * (dx / distance)
        val fy = forceMagnitude * (dy / distance)

        return Forces(fx, fy)
    }

    /**
     * Calculate atmospheric drag force
     * F_drag = 0.5 * ρ * v² * Cd * A
     */
    private fun calculateDrag(): Forces {
        val altitude = state.altitude
        if (altitude >= ATMOSPHERE_HEIGHT) return Forces(0.0, 0.0)

        // Exponential atmosphere model: ρ = ρ₀ * e^(-h/H)
        val density = AIR_DENSITY_SEA_LEVEL * exp(-altitude / SCALE_HEIGHT)

        // Velocity magnitude and direction
        val speed = sqrt(state.vx * state.vx + state.vy * state.vy)
        if (speed < 0.1) return Forces(0.0, 0.0)

        // Drag force magnitude: F = 0.5 * ρ * v² * Cd * A
        val dragMagnitude = 0.5 * density * speed * speed * DRAG_COEFFICIENT * ROCKET_CROSS_SECTION

        // Drag opposes motion (negative velocity direction)
        val fx = -dragMagnitude * (state.vx / speed)
        val fy = -dragMagnitude * (state.vy / speed)

        return Forces(fx, fy)
    }

    /**
     * Update derived properties from primary state
     */
    private fun updateDerivedProperties() {
        // Altitude (distance from surface)
        val distanceFromEarthCenter = sqrt(state.x * state.x + state.y * state.y)
        state.altitude = distanceFromEarthCenter - EARTH_RADIUS

        // Velocity magnitude
        state.velocity = sqrt(state.vx * state.vx + state.vy * state.vy)

        // Local gravity (varies with altitude)
        state.gravity = (G * EARTH_MASS) / (distanceFromEarthCenter * distanceFromEarthCenter)

        // Distance to Moon
        val moonX = EARTH_MOON_DISTANCE
        val moonY = 0.0
        val dx = moonX - state.x
        val dy = moonY - state.y
        state.distanceToMoon = sqrt(dx * dx + dy * dy)

        // Check if in orbit (simplified)
        val orbitalVelocity = sqrt(G * EARTH_MASS / distanceFromEarthCenter)
        state.inOrbit = abs(state.velocity - orbitalVelocity) < 500.0 && state.altitude > 200000.0
    }

    /**
     * Check if rocket has landed
     */
    private fun checkLanding() {
        // Landing on Earth
        if (state.altitude <= 0.0) {
            state.altitude = 0.0
            state.hasLanded = true
            state.vx = 0.0
            state.vy = 0.0
            state.isEngineOn = false
        }

        // Landing on Moon (within 5km of surface)
        val moonSurfaceDistance = state.distanceToMoon - MOON_RADIUS
        if (moonSurfaceDistance <= 5000.0 && state.velocity < 50.0) {
            state.hasLanded = true
            state.isEngineOn = false
        }
    }

    /**
     * Calculate required delta-V for Hohmann transfer to Moon
     */
    fun calculateHohmannDeltaV(): Double {
        val r1 = EARTH_RADIUS + 200000.0 // Low Earth Orbit (200km)
        val r2 = EARTH_MOON_DISTANCE

        val v1 = sqrt(G * EARTH_MASS / r1) // Orbital velocity at LEO

        // Transfer orbit calculations
        val vTransfer1 = sqrt(G * EARTH_MASS * (2.0/r1 - 2.0/(r1+r2)))
        val vTransfer2 = sqrt(G * EARTH_MASS * (2.0/r2 - 2.0/(r1+r2)))
        val vMoon = sqrt(G * EARTH_MASS / r2)

        val deltaV1 = abs(vTransfer1 - v1)
        val deltaV2 = abs(vMoon - vTransfer2)

        return deltaV1 + deltaV2
    }

    /**
     * Calculate orbital velocity at current altitude
     */
    fun getOrbitalVelocity(): Double {
        val r = sqrt(state.x * state.x + state.y * state.y)
        return sqrt(G * EARTH_MASS / r)
    }

    /**
     * Calculate escape velocity at current altitude
     */
    fun getEscapeVelocity(): Double {
        val r = sqrt(state.x * state.x + state.y * state.y)
        return sqrt(2.0 * G * EARTH_MASS / r)
    }

    /**
     * Get fuel percentage remaining
     */
    fun getFuelPercentage(): Double {
        return (state.fuelMass / FUEL_MASS_INITIAL) * 100.0
    }

    /**
     * Calculate time to reach Moon (simplified)
     */
    fun getTimeToMoon(): Double {
        if (state.velocity < 1.0) return Double.MAX_VALUE
        return state.distanceToMoon / state.velocity
    }

    /**
     * Get acceleration magnitude (felt g-force)
     */
    fun getAcceleration(): Double {
        return state.acceleration / 9.81 // in g's
    }

    /**
     * Calculate apoapsis (highest point of orbit)
     */
    fun getApoapsis(): Double {
        val r = sqrt(state.x * state.x + state.y * state.y)
        val v = state.velocity
        val specificOrbitalEnergy = (v * v / 2.0) - (G * EARTH_MASS / r)

        // For elliptical orbit
        val semiMajorAxis = -(G * EARTH_MASS) / (2.0 * specificOrbitalEnergy)

        // Calculate eccentricity (simplified)
        val h = state.x * state.vy - state.y * state.vx // Angular momentum per unit mass
        val eccentricity = sqrt(1.0 + (2.0 * specificOrbitalEnergy * h * h) / ((G * EARTH_MASS) * (G * EARTH_MASS)))

        val apoapsis = semiMajorAxis * (1.0 + eccentricity) - EARTH_RADIUS
        return if (apoapsis > 0 && apoapsis < EARTH_MOON_DISTANCE * 2) apoapsis else 0.0
    }

    /**
     * Calculate periapsis (lowest point of orbit)
     */
    fun getPeriapsis(): Double {
        val r = sqrt(state.x * state.x + state.y * state.y)
        val v = state.velocity
        val specificOrbitalEnergy = (v * v / 2.0) - (G * EARTH_MASS / r)

        val semiMajorAxis = -(G * EARTH_MASS) / (2.0 * specificOrbitalEnergy)
        val h = state.x * state.vy - state.y * state.vx
        val eccentricity = sqrt(1.0 + (2.0 * specificOrbitalEnergy * h * h) / ((G * EARTH_MASS) * (G * EARTH_MASS)))

        val periapsis = semiMajorAxis * (1.0 - eccentricity) - EARTH_RADIUS
        return if (periapsis > 0 && periapsis < EARTH_MOON_DISTANCE) periapsis else 0.0
    }

    // Control methods
    fun setThrottle(throttle: Double) {
        state.throttle = throttle.coerceIn(0.0, 1.0)
    }

    fun setEngineState(on: Boolean) {
        state.isEngineOn = on && state.fuelMass > 0
    }

    fun setAngle(angleRadians: Double) {
        state.angle = angleRadians
    }

    fun rotateRocket(deltaAngle: Double) {
        state.angle += deltaAngle
    }

    fun getCurrentState(): RocketState = state.copy()

    /**
     * Helper data class for forces
     */
    private data class Forces(val fx: Double, val fy: Double)
}

/**
 * TRAJECTORY CALCULATOR
 * Predicts future trajectory without running full simulation
 */
class TrajectoryPredictor(private val physicsEngine: RocketPhysicsEngine) {

    /**
     * Calculate trajectory points for visualization
     */
    fun predictTrajectory(
        steps: Int = 500,
        timeStep: Double = 1.0
    ): List<Pair<Double, Double>> {
        val trajectory = mutableListOf<Pair<Double, Double>>()

        // Create a copy of current state
        val testEngine = RocketPhysicsEngine()
        val currentState = physicsEngine.getCurrentState()

        // Run simulation forward
        repeat(steps) {
            val state = testEngine.update(timeStep)
            trajectory.add(Pair(state.x, state.y))

            if (state.hasLanded) return trajectory
        }

        return trajectory
    }

    /**
     * Calculate optimal burn time for reaching target altitude
     */
    fun calculateBurnTime(targetAltitude: Double, throttle: Double): Double {
        // Simplified calculation using rocket equation
        val deltaV = sqrt(2.0 * 9.81 * targetAltitude)
        val exhaustVel = RocketPhysicsEngine.EXHAUST_VELOCITY
        val massRatio = exp(deltaV / exhaustVel)

        val fuelNeeded = RocketPhysicsEngine.ROCKET_DRY_MASS * (massRatio - 1.0)
        val burnTime = fuelNeeded / (RocketPhysicsEngine.FUEL_CONSUMPTION_RATE * throttle)

        return burnTime
    }
}

/**
 * ORBITAL MECHANICS HELPER
 */
object OrbitalMechanics {

    /**
     * Calculate circular orbit velocity at given altitude
     */
    fun circularOrbitVelocity(altitude: Double): Double {
        val r = RocketPhysicsEngine.EARTH_RADIUS + altitude
        return sqrt(RocketPhysicsEngine.G * RocketPhysicsEngine.EARTH_MASS / r)
    }

    /**
     * Calculate orbital period (time for one complete orbit)
     */
    fun orbitalPeriod(altitude: Double): Double {
        val r = RocketPhysicsEngine.EARTH_RADIUS + altitude
        return 2.0 * PI * sqrt((r * r * r) / (RocketPhysicsEngine.G * RocketPhysicsEngine.EARTH_MASS))
    }

    /**
     * Calculate gravity turn angle for efficient ascent
     */
    fun gravityTurnAngle(altitude: Double, targetAltitude: Double): Double {
        val progress = (altitude / targetAltitude).coerceIn(0.0, 1.0)
        // Start vertical (90°), gradually turn to horizontal (0°)
        return (PI / 2.0) * (1.0 - progress)
    }

    /**
     * Calculate atmospheric density at altitude
     */
    fun atmosphericDensity(altitude: Double): Double {
        if (altitude >= RocketPhysicsEngine.ATMOSPHERE_HEIGHT) return 0.0
        return RocketPhysicsEngine.AIR_DENSITY_SEA_LEVEL *
                exp(-altitude / RocketPhysicsEngine.SCALE_HEIGHT)
    }

    /**
     * Calculate dynamic pressure (Q = 0.5 * ρ * v²)
     */
    fun dynamicPressure(altitude: Double, velocity: Double): Double {
        val density = atmosphericDensity(altitude)
        return 0.5 * density * velocity * velocity
    }
}

/**
 * AUTOPILOT SYSTEM
 * Automatic flight control for different mission profiles
 */
class RocketAutopilot(private val physicsEngine: RocketPhysicsEngine) {

    enum class FlightMode {
        MANUAL,
        VERTICAL_ASCENT,
        GRAVITY_TURN,
        ORBITAL_INSERTION,
        HOHMANN_TRANSFER,
        LANDING
    }

    private var currentMode = FlightMode.MANUAL
    private var targetAltitude = 200000.0 // Default: 200km orbit

    /**
     * Update autopilot - call every frame
     */
    fun update() {
        val state = physicsEngine.getCurrentState()

        when (currentMode) {
            FlightMode.VERTICAL_ASCENT -> {
                // Fly straight up until reaching velocity threshold
                physicsEngine.setAngle(PI / 2) // 90 degrees
                physicsEngine.setThrottle(1.0)

                if (state.altitude > 10000.0) {
                    currentMode = FlightMode.GRAVITY_TURN
                }
            }

            FlightMode.GRAVITY_TURN -> {
                // Gradually pitch over for efficient ascent
                val turnAngle = OrbitalMechanics.gravityTurnAngle(
                    state.altitude,
                    targetAltitude
                )
                physicsEngine.setAngle(turnAngle)
                physicsEngine.setThrottle(0.8)

                if (state.altitude >= targetAltitude * 0.9) {
                    currentMode = FlightMode.ORBITAL_INSERTION
                }
            }

            FlightMode.ORBITAL_INSERTION -> {
                // Burn to circularize orbit
                val targetVelocity = OrbitalMechanics.circularOrbitVelocity(state.altitude)

                if (state.velocity < targetVelocity) {
                    physicsEngine.setAngle(0.0) // Horizontal burn
                    physicsEngine.setThrottle(1.0)
                } else {
                    physicsEngine.setThrottle(0.0)
                    currentMode = FlightMode.MANUAL
                }
            }

            FlightMode.LANDING -> {
                // Suicide burn landing algorithm
                val timeToGround = sqrt(2.0 * state.altitude / state.gravity)
                val requiredDeceleration = state.velocity / timeToGround

                if (requiredDeceleration > state.gravity) {
                    physicsEngine.setAngle(PI / 2) // Point up
                    physicsEngine.setThrottle(1.0)
                } else {
                    physicsEngine.setThrottle(0.0)
                }
            }

            else -> {
                // Manual control - do nothing
            }
        }
    }

    fun setMode(mode: FlightMode, targetAlt: Double = 200000.0) {
        currentMode = mode
        targetAltitude = targetAlt

        if (mode != FlightMode.MANUAL) {
            physicsEngine.setEngineState(true)
        }
    }

    fun getMode(): FlightMode = currentMode
}