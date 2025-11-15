package com.exmple.aeroshift.custom

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import kotlin.math.sin
import kotlin.random.Random

class StarsView @JvmOverloads constructor(
    context: Context,
    attributeSet: AttributeSet?= null,
    defStyleAttr: Int = 0

): View(context, attributeSet, defStyleAttr) {

    private  val starPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFFFFFFFF.toInt()
        style = Paint.Style.FILL
    }

    private data class Star(
        val x:Float,
        val y:Float,
        val radius :Float,
        val bbaseAlpha:Int,
        val twinkleOffset :Float = Random.nextFloat() * Math.PI.toFloat() *2,
        val twinkleSpeed :Float = Random.nextFloat() *0.5f + 0.5f
    )
    private  val stars = mutableListOf<Star>()
    private  var time =0f
    private  var isAnimating = false

    init{
        generateStars()
    }

    private fun generateStars() {
        stars.clear()
        repeat(80){
           stars.add(
               Star(
                   x = Random.nextFloat(),
                   y = Random.nextFloat(),
                   radius = Random.nextFloat() * 2f + 1f,
                   bbaseAlpha = (Random.nextFloat() * 128 + 127).toInt(),
                   twinkleOffset = Random.nextFloat() * 08f + 04f
               )
           )
        }
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        if(w>0 && h> 0){
            generateStars()

        }
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        if(width ==0 || height ==0) return

        // Increment time for animation (assuming ~60fps)
        time +=0.016f
        stars.forEach { star->
            val x = star.x * width
            val y = star.y * height


            // Calculate twinkling alpha using sine wave
            val twinkle = sin((time * star.twinkleSpeed +  star.twinkleOffset).toDouble()).toFloat()
            val twinkAmount = (twinkle * 50f).toInt()
            val alpha  = (star.bbaseAlpha + twinkAmount).coerceIn(0,255)

            starPaint.alpha = alpha
            canvas.drawCircle(x, y, star.radius, starPaint)

        }
        if(isAnimating){
            invalidate()
        }
    }
    fun startAnimation(){
        isAnimating = true
        invalidate()
    }

    fun stopAnimation(){
        isAnimating  = false
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()
        startAnimation()
    }

    override fun onDetachedFromWindow() {
        super.onDetachedFromWindow()
        stopAnimation()
    }

}