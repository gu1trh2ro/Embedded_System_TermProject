package com.embedcreativity.bluetoothserial

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.os.Build
import androidx.core.app.ActivityCompat
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.content.Intent
import android.os.Bundle
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.ListView
import android.widget.Toast
import androidx.annotation.RequiresPermission
import androidx.appcompat.app.AppCompatActivity

class SelectDeviceActivity : AppCompatActivity() {

    private var mBtAdapter: BluetoothAdapter? = null
    private var mPairedDevices: Set<BluetoothDevice>? = null
    private lateinit var deviceListView: ListView

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.select_device_layout)

        deviceListView = findViewById(R.id.select_device_list)
        val btnRefresh = findViewById<Button>(R.id.select_device_refresh)

        mBtAdapter = BluetoothAdapter.getDefaultAdapter()

        if (mBtAdapter == null) {
            Toast.makeText(this, "블루투스를 지원하지 않는 기기입니다.", Toast.LENGTH_SHORT).show()
            finish()
            return
        }

        if (!mBtAdapter!!.isEnabled) {
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            startActivityForResult(enableBtIntent, 1)
        }

        btnRefresh.setOnClickListener {
            updateDeviceList()
        }

        updateDeviceList()
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    private fun updateDeviceList() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            return
        }

        mPairedDevices = mBtAdapter?.bondedDevices // 여기서 튕기지 않도록 위에서 막음
        val list: ArrayList<String> = ArrayList()

        if (mPairedDevices != null && mPairedDevices!!.isNotEmpty()) {
            for (device in mPairedDevices!!) {
                list.add("${device.name}\n${device.address}")
            }
        } else {
            Toast.makeText(this, "페어링된 기기가 없습니다.", Toast.LENGTH_SHORT).show()
        }

        val adapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, list)
        deviceListView.adapter = adapter
        deviceListView.setOnItemClickListener { _, _, position, _ ->
            val info = list[position]
            val address = info.substring(info.length - 17)

            val intent = Intent()
            intent.putExtra("device_address", address)
            setResult(RESULT_OK, intent)
            finish()
        }
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == 1) {
            if (resultCode == RESULT_OK) {
                updateDeviceList()
            } else {
                Toast.makeText(this, "블루투스를 활성화해야 합니다.", Toast.LENGTH_SHORT).show()
                finish()
            }
        }
    }
}