Aeroshift

Aeroshift is an Android prototype that simulates a reusable rocket. The rocket launches from Earth, travels to the Moon at the top-right corner, hovers for a few seconds, and returns back to the launch pad on Earth. The project currently uses animation-based motion but can be extended into a real physics-based simulation.

Demo

Add your demo video here.
You can place the video at: docs/media/demo.mp4
Then link it like this: [Demo Video](docs/media/demo.mp4)

You can also add a GIF preview:
![Demo](docs/media/demo.gif)

Features

* Earth to Moon rocket travel
* Moon fixed at top-right corner
* Launch, hover, auto-return, and landing sequence
* Altitude, velocity, and gravity HUD
* Launch and Auto-Land buttons
* ViewBinding enabled
* Animation-driven implementation

How to Run

1. Clone the repository:
   git clone [https://github.com/your-username/Aeroshift.git](https://github.com/your-username/Aeroshift.git)

2. Open the project in Android Studio.

3. Make sure ViewBinding is enabled in app/build.gradle:
   buildFeatures {
   viewBinding true
   }

4. Place required drawable assets:
   rocket.png
   moon.png
   launch_pad.png
   rocket_flame.png (optional)
   trajectory_path.png (optional)

5. Build and run the app.

Project Structure

app/

* src/main/java/com/exmple/aeroshift/MainActivity.kt
* src/main/res/layout/activity_main.xml
* src/main/res/drawable/
* src/main/res/values/
* build.gradle

Core Logic Overview

MainActivity.kt contains the main rocket simulation logic:

* launchSequence
* animateToMoon
* onReachedMoon
* returnToEarthSequence
* animateToEarth
* updateHUD
* updateStatus

The rocket uses ObjectAnimator and ValueAnimator to move and update HUD values during launch and landing.

Upgrading to a Physics Engine

To replace animation-based movement with real physics:

1. Create a physics update loop (16ms interval).
2. Update velocity and position using:
   velocity = velocity + acceleration * dt
   position = position + velocity * dt
3. Apply Earth and Moon gravity based on phase.
4. Update rocket image x/y using physics results.
5. Implement thrust control and landing deceleration.

ViewBinding Setup

In app/build.gradle:
android {
buildFeatures {
viewBinding true
}
}

In MainActivity:
private lateinit var binding: ActivityMainBinding
binding = ActivityMainBinding.inflate(layoutInflater)
setContentView(binding.root)

Troubleshooting

* If rocket position appears incorrect, calculate coordinates inside View.post or after layout measuring.
* Clamp HUD progress values between 0 and 100.
* If animations conflict, ensure no other animator is writing to the same properties.

License

MIT License

Contributing

You can contribute by opening issues or submitting pull requests.
