package com.example.aeroshift

import android.animation.ValueAnimator
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import androidx.appcompat.app.AppCompatActivity
import com.exmple.aeroshift.custom.RocketAutopilot
import com.exmple.aeroshift.custom.RocketPhysicsEngine
import com.exmple.aeroshift.databinding.ActivityMainBinding

import kotlin.math.*

/**
 * MainActivity with REAL PHYSICS ENGINE Integration
 * Features: Newtonian physics, orbital mechanics, fuel consumption, thrust, drag
 */
class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

    // Physics Engine
    private lateinit var physicsEngine: RocketPhysicsEngine
    private lateinit var autopilot: RocketAutopilot

    // Simulation loop
    private val simulationHandler = Handler(Looper.getMainLooper())
    private var isSimulationRunning = false
    private val SIMULATION_FPS = 60
    private val SIMULATION_INTERVAL = 1000L / SIMULATION_FPS // ~16ms

    // Display scaling (meters to pixels)
    private var pixelsPerMeter = 1.0
    private var screenCenterX = 0f
    private var screenCenterY = 0f

    // Coordinate references
    private var launchPadX = 0f
    private var launchPadY = 0f
    private var moonX = 0f
    private var moonY = 0f

    // Flight state
    private enum class FlightPhase {
        IDLE, LAUNCHING, IN_FLIGHT, ON_MOON, RETURNING, LANDED
    }
    private var currentPhase = FlightPhase.IDLE

    // Camera (view tracking)
    private var cameraScale = 1.0 // Zoom level
    private var cameraOffsetX = 0.0
    private var cameraOffsetY = 0.0

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Initialize physics
        physicsEngine = RocketPhysicsEngine()
        autopilot = RocketAutopilot(physicsEngine)

        setupUI()
        setupListeners()
        setupPhysics()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) {
            calculateCoordinates()
            calculateDisplayScaling()
        }
    }

    private fun calculateCoordinates() {
        binding.launchPadContainer.post {
            launchPadX = binding.launchPadContainer.x + (binding.launchPadContainer.width / 2f)
            launchPadY = binding.launchPadContainer.y
            screenCenterX = binding.root.width / 2f
            screenCenterY = binding.root.height / 2f
        }

        binding.moonImage.post {
            moonX = binding.moonImage.x + (binding.moonImage.width / 2f)
            moonY = binding.moonImage.y + (binding.moonImage.height / 2f)
        }
    }

    /**
     * Calculate scaling factor to map physics coordinates to screen pixels
     */
    private fun calculateDisplayScaling() {
        // Map Earth-Moon distance to screen width
        val screenWidth = binding.root.width.toDouble()
        val earthMoonDistance = RocketPhysicsEngine.EARTH_MOON_DISTANCE

        // Scale: distance from Earth center to moon position on screen
        val displayDistance = (moonX - screenCenterX).toDouble()
        pixelsPerMeter = displayDistance / earthMoonDistance
    }

    private fun setupUI() {
        updateRocketPosition(launchPadX, launchPadY)
        updateAllHUD()
        updateStatus(FlightPhase.IDLE)
    }

    private fun setupListeners() {
        binding.launchButton.setOnClickListener {
            if (currentPhase == FlightPhase.IDLE) {
                startLaunchSequence()
            }
        }

        binding.resetButton.setOnClickListener {
            resetSimulation()
        }

        binding.autoLandButton.setOnClickListener {
            if (currentPhase == FlightPhase.ON_MOON || currentPhase == FlightPhase.IN_FLIGHT) {
                startReturnSequence()
            }
        }
    }

    private fun setupPhysics() {
        physicsEngine.initialize()
    }

    /**
     * START LAUNCH SEQUENCE - Real physics simulation
     */
    private fun startLaunchSequence() {
        currentPhase = FlightPhase.LAUNCHING
        updateStatus(FlightPhase.LAUNCHING)

        binding.launchButton.isEnabled = false
        binding.autoLandButton.isEnabled = false

        // Show effects
        binding.rocketFlame.visibility = View.VISIBLE
        binding.rocketFlame.animate().alpha(1f).setDuration(200).start()
        binding.trajectoryPath.visibility = View.VISIBLE

        // Initialize physics
        physicsEngine.initialize()
        physicsEngine.setEngineState(true)
        physicsEngine.setThrottle(1.0) // Full throttle

        // Start simulation loop
        startPhysicsSimulation()

        // Use autopilot for launch
        autopilot.setMode(RocketAutopilot.FlightMode.VERTICAL_ASCENT)
    }

    /**
     * PHYSICS SIMULATION LOOP - Called every frame
     */
    private val physicsSimulationRunnable = object : Runnable {
        override fun run() {
            if (!isSimulationRunning) return

            // Update physics (60 FPS)
            val state = physicsEngine.update(RocketPhysicsEngine.TIME_STEP)

            // Update autopilot
            autopilot.update()

            // Update display
            updateRocketFromPhysics(state)
            updateAllHUD()

            // Check mission progress
            checkMissionProgress(state)

            // Continue simulation
            if (isSimulationRunning) {
                simulationHandler.postDelayed(this, SIMULATION_INTERVAL)
            }
        }
    }

    private fun startPhysicsSimulation() {
        if (!isSimulationRunning) {
            isSimulationRunning = true
            simulationHandler.post(physicsSimulationRunnable)
        }
    }

    private fun stopPhysicsSimulation() {
        isSimulationRunning = false
        simulationHandler.removeCallbacks(physicsSimulationRunnable)
    }

    /**
     * Update rocket position on screen from physics state
     */
    private fun updateRocketFromPhysics(state: RocketPhysicsEngine.RocketState) {
        // Convert physics coordinates (meters) to screen coordinates (pixels)
        val physicsX = state.x
        val physicsY = state.y

        // Map to screen (Earth center = screen center initially)
        var screenX = screenCenterX + (physicsX * pixelsPerMeter).toFloat()
        var screenY = screenCenterY - (physicsY * pixelsPerMeter).toFloat() // Invert Y

        // Camera follow (zoom out as rocket gets higher)
        val altitude = state.altitude
        if (altitude > 50000.0) {
            // Calculate auto-zoom based on altitude
            cameraScale = max(0.1, 1.0 - (altitude / RocketPhysicsEngine.EARTH_MOON_DISTANCE) * 0.9)

            // Apply camera scale
            screenX = screenCenterX + ((physicsX * pixelsPerMeter * cameraScale).toFloat())
            screenY = screenCenterY - ((physicsY * pixelsPerMeter * cameraScale).toFloat())
        }

        // Keep rocket on screen (clamp)
        screenX = screenX.coerceIn(0f, binding.root.width.toFloat())
        screenY = screenY.coerceIn(0f, binding.root.height.toFloat())

        // Update rocket position
        binding.rocketImage.x = screenX - binding.rocketImage.width / 2
        binding.rocketImage.y = screenY - binding.rocketImage.height / 2

        // Update rocket rotation (angle in radians to degrees)
        val angleDegrees = Math.toDegrees(state.angle).toFloat() - 90f // Adjust for sprite orientation
        binding.rocketImage.rotation = -angleDegrees // Negative for screen coordinates

        // Update flame based on throttle
        val flameAlpha = if (state.isEngineOn) state.throttle.toFloat() else 0f
        binding.rocketFlame.alpha = flameAlpha
    }

    /**
     * Check mission progress and phase transitions
     */
    private fun checkMissionProgress(state: RocketPhysicsEngine.RocketState) {
        when (currentPhase) {
            FlightPhase.LAUNCHING -> {
                // Transition to in-flight after leaving atmosphere
                if (state.altitude > 100000.0) {
                    currentPhase = FlightPhase.IN_FLIGHT
                    updateStatus(FlightPhase.IN_FLIGHT)
                }
            }

            FlightPhase.IN_FLIGHT -> {
                // Check if reached moon vicinity (within 50km)
                if (state.distanceToMoon < 50000.0) {
                    currentPhase = FlightPhase.ON_MOON
                    updateStatus(FlightPhase.ON_MOON)
                    binding.autoLandButton.isEnabled = true

                    // Auto-hover (reduce velocity)
                    physicsEngine.setThrottle(0.5)
                    autopilot.setMode(RocketAutopilot.FlightMode.MANUAL)
                }

                // Check if ran out of fuel mid-flight
                if (state.fuelMass <= 0 && !state.hasLanded) {
                    physicsEngine.setEngineState(false)
                }
            }

            FlightPhase.ON_MOON -> {
                // Stay in moon vicinity
                if (state.distanceToMoon > 100000.0) {
                    currentPhase = FlightPhase.IN_FLIGHT
                    updateStatus(FlightPhase.IN_FLIGHT)
                }
            }

            FlightPhase.RETURNING -> {
                // Check if back in Earth atmosphere
                if (state.altitude < 100000.0 && state.altitude > 1000.0) {
                    // Prepare for landing
                    autopilot.setMode(RocketAutopilot.FlightMode.LANDING)
                }
            }

            else -> {}
        }

        // Check landing
        if (state.hasLanded) {
            onLanding()
        }
    }

    /**
     * Start return to Earth sequence
     */
    private fun startReturnSequence() {
        currentPhase = FlightPhase.RETURNING
        updateStatus(FlightPhase.RETURNING)
        binding.autoLandButton.isEnabled = false

        // Point toward Earth and burn
        val state = physicsEngine.getCurrentState()
        val earthDirection = atan2(-state.y, -state.x)

        physicsEngine.setAngle(earthDirection)
        physicsEngine.setThrottle(0.8)
        physicsEngine.setEngineState(true)

        binding.rocketFlame.visibility = View.VISIBLE
        binding.rocketFlame.animate().alpha(1f).setDuration(200).start()
    }

    /**
     * Landing callback
     */
    private fun onLanding() {
        currentPhase = FlightPhase.LANDED
        updateStatus(FlightPhase.LANDED)

        stopPhysicsSimulation()

        // Hide flame
        binding.rocketFlame.animate().alpha(0f).setDuration(300).withEndAction {
            binding.rocketFlame.visibility = View.GONE
        }.start()

        // Landing bounce animation
        binding.rocketImage.animate()
            .scaleX(1.1f)
            .scaleY(0.9f)
            .setDuration(100)
            .withEndAction {
                binding.rocketImage.animate()
                    .scaleX(1f)
                    .scaleY(1f)
                    .setDuration(100)
                    .withEndAction {
                        binding.rocketImage.postDelayed({
                            currentPhase = FlightPhase.IDLE
                            updateStatus(FlightPhase.IDLE)
                            binding.launchButton.isEnabled = true
                        }, 1000)
                    }
                    .start()
            }
            .start()
    }

    /**
     * Reset simulation
     */
    private fun resetSimulation() {
        stopPhysicsSimulation()

        currentPhase = FlightPhase.IDLE
        physicsEngine.initialize()
        autopilot.setMode(RocketAutopilot.FlightMode.MANUAL)

        // Reset UI
        cameraScale = 1.0
        updateRocketPosition(launchPadX, launchPadY)
        binding.rocketImage.rotation = 0f
        binding.rocketImage.scaleX = 1f
        binding.rocketImage.scaleY = 1f

        binding.rocketFlame.visibility = View.GONE
        binding.rocketFlame.alpha = 0f
        binding.trajectoryPath.visibility = View.INVISIBLE

        updateAllHUD()
        updateStatus(FlightPhase.IDLE)

        binding.launchButton.isEnabled = true
        binding.autoLandButton.isEnabled = false
    }

    /**
     * Update rocket position manually (for initial placement)
     */
    private fun updateRocketPosition(x: Float, y: Float) {
        binding.rocketImage.x = x - binding.rocketImage.width / 2
        binding.rocketImage.y = y
    }

    /**
     * Update ALL HUD indicators from physics state
     */
    private fun updateAllHUD() {
        val state = physicsEngine.getCurrentState()

        // Altitude (convert meters to km)
        val altitudeKm = state.altitude / 1000.0
        binding.altitudeValue.text = String.format("%.0f", altitudeKm)
        val altitudeProgress = ((altitudeKm / 384400.0) * 100).toInt().coerceIn(0, 100)
        binding.altitudeProgress.progress = altitudeProgress

        // Velocity (convert m/s to km/s)
        val velocityKms = state.velocity / 1000.0
        binding.velocityValue.text = String.format("%.1f", velocityKms)
        val velocityProgress = ((velocityKms / 15.0) * 100).toInt().coerceIn(0, 100)
        binding.velocityProgress.progress = velocityProgress

        // Gravity (local gravity in m/s²)
        binding.gravityValue.text = String.format("%.2f", state.gravity)

    }

    /**
     * Update status text
     */
    private fun updateStatus(phase: FlightPhase) {
        val state = physicsEngine.getCurrentState()
        val statusText = when (phase) {
            FlightPhase.IDLE -> "READY FOR LAUNCH"
            FlightPhase.LAUNCHING -> "LAUNCHING - ${String.format("%.0f", physicsEngine.getFuelPercentage())}% FUEL"
            FlightPhase.IN_FLIGHT -> {
                if (state.inOrbit) {
                    "IN ORBIT - ${String.format("%.0f", state.distanceToMoon / 1000.0)} km to Moon"
                } else {
                    "IN FLIGHT - ${String.format("%.1f", state.velocity / 1000.0)} km/s"
                }
            }
            FlightPhase.ON_MOON -> "MOON ORBIT - ${String.format("%.0f", state.distanceToMoon / 1000.0)} km"
            FlightPhase.RETURNING -> "RETURNING TO EARTH - ${String.format("%.0f", state.altitude / 1000.0)} km"
            FlightPhase.LANDED -> "LANDED SAFELY"
        }
        binding.statusText.text = statusText
    }

    override fun onPause() {
        super.onPause()
        stopPhysicsSimulation()
    }

    override fun onDestroy() {
        super.onDestroy()
        stopPhysicsSimulation()
    }
}

