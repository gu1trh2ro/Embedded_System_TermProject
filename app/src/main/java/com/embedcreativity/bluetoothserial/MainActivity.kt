package com.embedcreativity.bluetoothserial

import android.Manifest
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val btnConnect = findViewById<Button>(R.id.btnConnect)

        // 앱 실행 시 권한 요청
        checkPermissions()

        btnConnect.setOnClickListener {
            // 안드로이드 12 이상일 경우 BLUETOOTH_CONNECT 권한이 있는지 확인
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
                ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                Toast.makeText(this, "블루투스 권한이 필요합니다.", Toast.LENGTH_SHORT).show()
                checkPermissions()
            } else {
                // 장치 선택 화면(SelectDeviceActivity)으로 이동 (요청 코드 100)
                val intent = Intent(this, SelectDeviceActivity::class.java)
                startActivityForResult(intent, 100)
            }
        }

        // 시뮬레이션 모드 (건너뛰기) 버튼
        findViewById<Button>(R.id.btnSimulate).setOnClickListener {
            val intent = Intent(this, ControlActivity::class.java)
            // device_address 없이 시작하면 ControlActivity에서 시뮬레이션 모드로 인식
            startActivity(intent)
        }
    }

    // 장치 선택 화면에서 돌아왔을 때 실행되는 콜백
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)

        // SelectDeviceActivity에서 장치를 성공적으로 선택한 경우
        if (requestCode == 100 && resultCode == Activity.RESULT_OK) {
            val address = data?.getStringExtra("device_address")

            if (address != null) {
                // 선택한 장치의 주소(MAC Address)를 가지고 실시간 모니터링 화면(ControlActivity)으로 이동
                val intent = Intent(this, ControlActivity::class.java)
                intent.putExtra("device_address", address)
                startActivity(intent)
            } else {
                Toast.makeText(this, "장치 주소를 가져오지 못했습니다.", Toast.LENGTH_SHORT).show()
            }
        }
    }

    private fun checkPermissions() {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION
            )
        }

        val missingPermissions = permissions.filter {
            ActivityCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (missingPermissions.isNotEmpty()) {
            ActivityCompat.requestPermissions(this, missingPermissions.toTypedArray(), 1)
        }
    }
}