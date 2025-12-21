package com.embedcreativity.bluetoothserial

import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.graphics.Color
import android.os.*
import android.widget.Button
import android.widget.Switch
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import android.app.AlertDialog
import android.widget.EditText
import android.widget.LinearLayout
import android.text.InputType
import android.view.Gravity

class ControlActivity : AppCompatActivity() {

    private fun updateTimerText(totalSeconds: Long) {
        val hours = totalSeconds / 3600
        val minutes = (totalSeconds % 3600) / 60
        val seconds = totalSeconds % 60
        var timeString = String.format("오늘의 집중 시간: %02d:%02d:%02d", hours, minutes, seconds)

        // Add Goal Progress
        if (dailyGoalSeconds > 0) {
            val progress = (totalSeconds.toFloat() / dailyGoalSeconds.toFloat()) * 100
            val goalMinsTotal = dailyGoalSeconds / 60
            val goalHours = goalMinsTotal / 60
            val goalMins = goalMinsTotal % 60
            
            var goalString = ""
            if (goalHours > 0) goalString += "${goalHours}시간 "
            if (goalMins > 0 || goalHours.toInt() == 0) goalString += "${goalMins}분"
            
            timeString += String.format("\n오늘의 목표: %s / 목표 달성률: %.0f%%", goalString.trim(), progress)

            // Achievement Check (Trigger only once per sessions when threshold crossed, or handle state refresh)
            if (totalSeconds >= dailyGoalSeconds && !isCelebrated && isOccupied) {
               showAchievementState()
            }
        }
        focusTimerText.text = timeString
    }

    private var isCelebrated = false // To prevent repetitive celebration in one session

    private fun showAchievementState() {
        if (isCelebrated) return
        isCelebrated = true

        runOnUiThread {
            // 1. Show Trophy & Gold
            statusIcon.setImageResource(R.drawable.ic_status_trophy)
            statusIcon.imageTintList = null // Clear tint to show original image colors
            statusCircleBg.backgroundTintList = android.content.res.ColorStateList.valueOf(Color.parseColor("#FFD700")) // Gold
            statusDisplay.text = "목표 달성!"

            // 2. Play Sound or Animation (Optional - here just UI change)

            // 3. Revert after 5 seconds
            Handler(Looper.getMainLooper()).postDelayed({
                updateUI("OCCUPIED") // Revert to normal state (but timer will keep counting)
                // Note: updateUI will reset text/icon, but isCelebrated remains true until reset manually or new day
            }, 5000)
        }
    }

    private fun showGoalSettingDialog() {
        val builder = AlertDialog.Builder(this)
        builder.setTitle("목표 시간 설정")
        builder.setMessage("오늘의 목표 시간을 시/분 단위로 입력하세요.")

        // Layout for Inputs
        val layout = LinearLayout(this)
        layout.orientation = LinearLayout.HORIZONTAL
        layout.gravity = Gravity.CENTER
        layout.setPadding(32, 32, 32, 32)

        val inputHours = EditText(this)
        inputHours.hint = "시간"
        inputHours.inputType = InputType.TYPE_CLASS_NUMBER
        inputHours.gravity = Gravity.CENTER
        inputHours.layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f)

        val inputMinutes = EditText(this)
        inputMinutes.hint = "분"
        inputMinutes.inputType = InputType.TYPE_CLASS_NUMBER
        inputMinutes.gravity = Gravity.CENTER
        inputMinutes.layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f)

        layout.addView(inputHours)
        layout.addView(inputMinutes)
        builder.setView(layout)

        builder.setPositiveButton("설정") { _, _ ->
            val hoursText = inputHours.text.toString()
            val minutesText = inputMinutes.text.toString()
            
            val hours = if (hoursText.isNotEmpty()) hoursText.toLong() else 0
            val minutes = if (minutesText.isNotEmpty()) minutesText.toLong() else 0
            
            if (hours > 0 || minutes > 0) {
                val totalMinutes = (hours * 60) + minutes
                dailyGoalSeconds = totalMinutes * 60
                
                var goalMsg = "목표가 "
                if (hours > 0) goalMsg += "${hours}시간 "
                if (minutes > 0) goalMsg += "${minutes}분"
                goalMsg += "으로 설정되었습니다."

                Toast.makeText(this, goalMsg, Toast.LENGTH_SHORT).show()
                updateTimerText(totalFocusTimeSeconds) // Refresh display immediately
                isCelebrated = false // Reset celebration flag for new goal
            }
        }
        builder.setNegativeButton("취소") { dialog, _ -> dialog.cancel() }
        builder.show()
    }

    private var bluetoothService: BluetoothService? = null
    private var isBound = false
    private lateinit var statusDisplay: TextView
    private lateinit var connectionStateDisplay: TextView
    private lateinit var statusCircleBg: android.view.View
    private lateinit var statusIcon: android.widget.ImageView
    private lateinit var focusTimerText: TextView

    private var isAdminMode = false // Default: User Mode

    private var lastReceivedData: String = "WAITING" // Default state
    private var validSeatState: String = "WAITING" // Last known stable seat state (OCCUPIED/EMPTY)
    private var preRestroomState: String = "WAITING" // State BEFORE entering Restroom Mode
    private var isRestroomMode = false // Flag for Restroom/Away Mode

    private lateinit var iconRestroom: android.widget.ImageView
    private lateinit var iconLightStatus: android.widget.ImageView
    private lateinit var rootLayout: androidx.constraintlayout.widget.ConstraintLayout
    private var isLightOn = false // Track light status (ON/OFF)

    // Focus Timer Variables
    private var totalFocusTimeSeconds: Long = 0
    private var dailyGoalSeconds: Long = 0 // Goal in seconds (0 = disabled)
    private var currentSessionStartTime: Long = 0
    private var isOccupied = false
    private val timerHandler = Handler(Looper.getMainLooper())
    private val timerRunnable = object : Runnable {
        override fun run() {
            if (isOccupied) {
                // 현재 세션 시간 계산 (현재 시간 - 시작 시간)
                val currentSessionDuration = (System.currentTimeMillis() - currentSessionStartTime) / 1000
                // 총 시간 = 이전 누적 시간 + 현재 세션 시간
                val displayTime = totalFocusTimeSeconds + currentSessionDuration
                updateTimerText(displayTime)
                timerHandler.postDelayed(this, 1000)
            }
        }
    }

    // 데이터를 받아서 화면을 실시간으로 교체하는 핸들러
    private val mHandler = object : Handler(Looper.getMainLooper()) {
        override fun handleMessage(msg: Message) {
            // BluetoothService에서 MESSAGE_READ(1)로 보낸 데이터를 처리
            if (msg.what == 1) {
                val readMessage = msg.obj as String
                // Debug Toast (Optional: Remove later)
                // Toast.makeText(this@ControlActivity, "Recv: $readMessage", Toast.LENGTH_SHORT).show()
                
                // 수신된 텍스트의 앞뒤 공백 및 줄바꿈 제거 후 상태 업데이트
                updateUI(readMessage.trim())
            } else if (msg.what == BluetoothService.MESSAGE_CONNECTED) {
                connectionStateDisplay.text = "장치 연결됨 (Connected)"
                Toast.makeText(this@ControlActivity, "블루투스 장치에 연결되었습니다.", Toast.LENGTH_SHORT).show()
                
                // Change state to EMPTY immediately upon connection
                updateUI("EMPTY")
            } else if (msg.what == BluetoothService.MESSAGE_CONNECTION_FAILED) {
                connectionStateDisplay.text = "연결 실패 (Failed)"
                connectionStateDisplay.setTextColor(Color.RED)
            } else if (msg.what == BluetoothService.MESSAGE_CONNECTION_FAILED) {
                connectionStateDisplay.text = "연결 실패 (Failed)"
                connectionStateDisplay.setTextColor(Color.RED)
                Toast.makeText(this@ControlActivity, "연결에 실패했습니다.\n다른 기기가 연결 중인지 확인하세요.", Toast.LENGTH_LONG).show()
                BluetoothService.m_isConnected = false
            }
        }
    }


    // 최신 데이터에 따라 화면 텍스트를 완전히 교체하는 함수
    private fun updateUI(data: String) {
        val colorOccupied = androidx.core.content.ContextCompat.getColor(this, R.color.status_occupied)
        val colorEmpty = androidx.core.content.ContextCompat.getColor(this, R.color.status_empty)
        val colorSuspicious = androidx.core.content.ContextCompat.getColor(this, R.color.status_suspicious)
        val colorDefault = androidx.core.content.ContextCompat.getColor(this, R.color.colorPrimary)

        // 1. Handle AWAY / BACK Signals
        if (data == "AWAY") {
            if (!isRestroomMode) {
                isRestroomMode = true
                preRestroomState = validSeatState // Save last valid seat state
                
                // Fade In Animation
                iconRestroom.apply {
                    alpha = 0f
                    visibility = android.view.View.VISIBLE
                    animate().alpha(1f).setDuration(300).start()
                }
                
                if (!isAdminMode) {
                    Toast.makeText(this, "화장실 모드: 활성화 (자리 비움)", Toast.LENGTH_SHORT).show()
                }
                processState("SUSPICIOUS")
            }
            return
        }
        
        if (data == "BACK") {
            if (isRestroomMode) {
                isRestroomMode = false
                
                // Fade Out Animation
                iconRestroom.animate().alpha(0f).setDuration(300).withEndAction {
                    iconRestroom.visibility = android.view.View.GONE
                }.start()
                
                if (!isAdminMode) {
                    Toast.makeText(this, "화장실 모드: 해제 (복귀)", Toast.LENGTH_SHORT).show()
                }
                
                // RESTORE to the state before Restroom Mode
                processState(preRestroomState)
            }
            return
        }

        // New Rule: OCCUPIED or EMPTY also clears Restroom Mode
        if (data == "OCCUPIED" || data == "EMPTY") {
            if (isRestroomMode) {
                isRestroomMode = false
                iconRestroom.animate().alpha(0f).setDuration(300).withEndAction {
                    iconRestroom.visibility = android.view.View.GONE
                }.start()
                // Proceed to processState(data) below to update to the specific state
            }
        }

        // 2. Handle Light Control (ON/OFF)
        if (data == "ON") {
            isLightOn = true
            // Show icon only if in OCCUPIED state
            if (validSeatState == "OCCUPIED" && !isRestroomMode) {
                 iconLightStatus.visibility = android.view.View.VISIBLE
            }
            return
        }
        if (data == "OFF") {
            isLightOn = false
            iconLightStatus.visibility = android.view.View.GONE
            return
        }

        // 3. Normal Processing
        processState(data) 
    }

    private fun processState(data: String) {
        // Save last data for immediate refresh on mode switch
        lastReceivedData = data

        // Only update validSeatState if the data is a stable seat status
        if (data == "OCCUPIED" || data == "EMPTY") {
            validSeatState = data
        }

        val colorOccupied = androidx.core.content.ContextCompat.getColor(this, R.color.status_occupied)
        val colorEmpty = androidx.core.content.ContextCompat.getColor(this, R.color.status_empty)
        val colorSuspicious = androidx.core.content.ContextCompat.getColor(this, R.color.status_suspicious)
        val colorWaiting = androidx.core.content.ContextCompat.getColor(this, R.color.status_waiting)
        val colorDefault = androidx.core.content.ContextCompat.getColor(this, R.color.colorPrimary)

        when (data) {
            "OCCUPIED" -> {
                if (!isOccupied) {
                    // Start Timer
                    isOccupied = true
                    currentSessionStartTime = System.currentTimeMillis()
                    timerHandler.post(timerRunnable)
                }

                val targetText = if (isAdminMode) "사용 중 좌석" else "집중 중"
                animateStatusChange(targetText, colorOccupied, R.drawable.ic_status_occupied, R.color.status_occupied_bg)
                
                // If Light was already ON, ensure icon is visible when returning to OCCUPIED
                if (isLightOn) {
                    iconLightStatus.visibility = android.view.View.VISIBLE
                }
            }
            "EMPTY" -> {
                if (isOccupied) {
                    isOccupied = false
                    timerHandler.removeCallbacks(timerRunnable)
                    val sessionDuration = (System.currentTimeMillis() - currentSessionStartTime) / 1000
                    totalFocusTimeSeconds += sessionDuration
                    updateTimerText(totalFocusTimeSeconds)
                }

                val targetText = if (isAdminMode) "빈 좌석" else "빈 좌석"
                animateStatusChange(targetText, colorEmpty, R.drawable.ic_status_empty, R.color.status_empty_bg)
                // Hide light status in EMPTY state
                iconLightStatus.visibility = android.view.View.GONE
            }
            "SUSPICIOUS" -> {
                if (isOccupied) {
                    isOccupied = false
                    timerHandler.removeCallbacks(timerRunnable)
                    val sessionDuration = (System.currentTimeMillis() - currentSessionStartTime) / 1000
                    totalFocusTimeSeconds += sessionDuration
                    updateTimerText(totalFocusTimeSeconds)
                }

                val targetText = if (isAdminMode) "자리 비움" else "부재중"
                animateStatusChange(targetText, colorSuspicious, R.drawable.ic_status_suspicious, R.color.status_suspicious_bg)
            }
            "WAITING" -> {
                 if (isOccupied) {
                    // Stop Timer and Accumulate if previously running
                    isOccupied = false
                    timerHandler.removeCallbacks(timerRunnable)
                    val sessionDuration = (System.currentTimeMillis() - currentSessionStartTime) / 1000
                    totalFocusTimeSeconds += sessionDuration
                    updateTimerText(totalFocusTimeSeconds)
                }
                
                // Grey Theme for Waiting Connection
                animateStatusChange("연결 대기", colorWaiting, R.drawable.ic_bluetooth, R.color.status_waiting_bg)
            }
            else -> {
                if (isOccupied) {
                    // Stop Timer and Accumulate
                    isOccupied = false
                    timerHandler.removeCallbacks(timerRunnable)
                    val sessionDuration = (System.currentTimeMillis() - currentSessionStartTime) / 1000
                    totalFocusTimeSeconds += sessionDuration
                    updateTimerText(totalFocusTimeSeconds)
                }

                animateStatusChange("대기 중", colorDefault, R.drawable.ic_status_empty, R.color.colorBackground)
            }
        }
    }

    // Helper function for smooth UI transitions
    private fun animateStatusChange(newText: String, newColor: Int, newIconRes: Int, backgroundColorResId: Int) {
        val newBackgroundColor = androidx.core.content.ContextCompat.getColor(this, backgroundColorResId)

        // 1. Text Animation (Fade Out -> Change -> Fade In)
        if (statusDisplay.text.toString() != newText) {
             statusDisplay.animate().alpha(0f).setDuration(150).withEndAction {
                 statusDisplay.text = newText
                 statusDisplay.animate().alpha(1f).setDuration(150).start()
             }.start()
        }

        // 2. Icon Animation (Scale Down -> Change -> Scale Up)
        statusIcon.animate().scaleX(0.8f).scaleY(0.8f).setDuration(150).withEndAction {
            statusIcon.setImageResource(newIconRes)
            statusIcon.imageTintList = android.content.res.ColorStateList.valueOf(Color.WHITE)
            statusIcon.animate().scaleX(1f).scaleY(1f).setDuration(150).start()
        }.start()

        // 3. Status Circle Color Animation
        val currentCircleColor = statusCircleBg.backgroundTintList?.defaultColor ?: Color.GRAY
        if (currentCircleColor != newColor) {
            val colorAnimation = android.animation.ValueAnimator.ofObject(android.animation.ArgbEvaluator(), currentCircleColor, newColor)
            colorAnimation.duration = 300
            colorAnimation.addUpdateListener { animator ->
                val color = animator.animatedValue as Int
                statusCircleBg.backgroundTintList = android.content.res.ColorStateList.valueOf(color)
                
                // Animate Action Bar Color
                supportActionBar?.setBackgroundDrawable(android.graphics.drawable.ColorDrawable(color))
                // Animate Window Status Bar Color
                window.statusBarColor = color
            }
            colorAnimation.start()
        }
        
        // 4. Root Background Color Animation
        val currentBgColor = (rootLayout.background as? android.graphics.drawable.ColorDrawable)?.color ?: androidx.core.content.ContextCompat.getColor(this, R.color.colorBackground)
        if (currentBgColor != newBackgroundColor) {
             val bgAnimation = android.animation.ValueAnimator.ofObject(android.animation.ArgbEvaluator(), currentBgColor, newBackgroundColor)
             bgAnimation.duration = 300
             bgAnimation.addUpdateListener { animator ->
                 rootLayout.setBackgroundColor(animator.animatedValue as Int)
             }
             bgAnimation.start()
        }
    }

    private val connection = object : ServiceConnection {
        override fun onServiceConnected(className: ComponentName, service: IBinder) {
            val binder = service as BluetoothService.MyBinder
            bluetoothService = binder.getService()
            isBound = true

            // 서비스에 핸들러 등록 (데이터 수신 준비)
            BluetoothService.handler = mHandler

            // MainActivity로부터 넘겨받은 MAC 주소로 연결 시도
            val address = intent.getStringExtra("device_address")
            if (address != null) {
                connectionStateDisplay.text = "장치 연결 시도 중..."
                bluetoothService?.connectToDevice(address)
            } else {
                // 주소가 없으면 시뮬레이션 모드 (연결 안함)
                connectionStateDisplay.text = "시뮬레이션 모드 (연결 없음)"
                BluetoothService.m_isConnected = false
            }
        }

        override fun onServiceDisconnected(arg0: ComponentName) {
            isBound = false
            connectionStateDisplay.text = "연결 끊김 (Connection Lost)"
            Toast.makeText(this@ControlActivity, "블루투스 연결이 끊어졌습니다.", Toast.LENGTH_LONG).show()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.control_layout)

        // UI 요소 연결 (XML의 ID와 일치)
        statusDisplay = findViewById(R.id.status_text_display)
        connectionStateDisplay = findViewById(R.id.conn_state_text)
        statusCircleBg = findViewById(R.id.status_circle_bg)
        statusIcon = findViewById(R.id.status_icon)
        focusTimerText = findViewById(R.id.text_focus_timer)
        focusTimerText = findViewById(R.id.text_focus_timer)
        iconRestroom = findViewById(R.id.icon_restroom)
        iconLightStatus = findViewById(R.id.icon_light_status)
        rootLayout = findViewById(R.id.root_layout)

        // Set Goal Setting Click Listener
        val setGoalText = findViewById<TextView>(R.id.text_set_goal)
        setGoalText.setOnClickListener {
            if (!isAdminMode) {
                showGoalSettingDialog()
            }
        }

        // --- Admin Mode Toggle ---
        val switchMode = findViewById<Switch>(R.id.switch_mode)
        val btnReset = findViewById<Button>(R.id.btnSend)

        // Initial State (User Mode)
        btnReset.visibility = android.view.View.GONE
        focusTimerText.visibility = android.view.View.VISIBLE

        switchMode.setOnCheckedChangeListener { _, isChecked ->
            isAdminMode = isChecked
            if (isAdminMode) {
                // Admin Mode
                btnReset.visibility = android.view.View.VISIBLE
                focusTimerText.visibility = android.view.View.GONE
                setGoalText.visibility = android.view.View.GONE
                Toast.makeText(this, "관리자 모드: ON", Toast.LENGTH_SHORT).show()
                updateUI(lastReceivedData) // Immediate Refresh
            } else {
                // User Mode
                btnReset.visibility = android.view.View.GONE
                focusTimerText.visibility = android.view.View.VISIBLE
                setGoalText.visibility = android.view.View.VISIBLE
                Toast.makeText(this, "사용자 모드 (집중 타이머)", Toast.LENGTH_SHORT).show()
                updateUI(lastReceivedData) // Immediate Refresh
            }
        }
        // -------------------------

        // --- 시뮬레이션 모드 버튼 설정 ---
        findViewById<Button>(R.id.btnSimOccupied).setOnClickListener {
            updateUI("OCCUPIED")
        }
        findViewById<Button>(R.id.btnSimEmpty).setOnClickListener {
            updateUI("EMPTY")
        }
        findViewById<Button>(R.id.btnSimSuspicious).setOnClickListener {
            updateUI("SUSPICIOUS")
        }
        // --------------------------------

        // val btnReset = findViewById<Button>(R.id.btnSend) // Already declared above
        btnReset.text = "수동 리셋 (R 전송)"

        // BluetoothService 바인딩 시작
        val intent = Intent(this, BluetoothService::class.java)
        bindService(intent, connection, Context.BIND_AUTO_CREATE)

        btnReset.setOnClickListener {
            if (isBound && BluetoothService.m_isConnected) {
                // 보드로 'R' 문자를 보내 리셋 명령 수행
                bluetoothService?.sendCommand("R")
                Toast.makeText(this, "리셋 명령을 보냈습니다.", Toast.LENGTH_SHORT).show()
            } else {
                Toast.makeText(this, "연결 상태를 확인하세요.", Toast.LENGTH_SHORT).show()
            }
        }

        // Initialize UI with default state
        updateUI(lastReceivedData)
    }

    override fun onDestroy() {
        super.onDestroy()
        if (isBound) {
            unbindService(connection)
            isBound = false
        }
    }
}