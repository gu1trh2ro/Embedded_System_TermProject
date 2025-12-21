package com.embedcreativity.bluetoothserial

import android.Manifest
import android.app.Service
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.Intent
import android.os.*
import android.util.Log
import androidx.annotation.RequiresPermission
import java.io.IOException
import java.io.InputStream
import java.util.*

class BluetoothService : Service() {

    private val myBinder = MyBinder()
    private var receiveThread: ReceiveThread? = null

    companion object {
        var m_myUUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        var m_bluetoothSocket: BluetoothSocket? = null
        var m_isConnected: Boolean = false
        lateinit var m_address: String

        // 데이터 수신을 위한 핸들러 (Activity와 통신용)
        // 데이터 수신을 위한 핸들러 (Activity와 통신용)
        var handler: Handler? = null
        const val MESSAGE_READ = 1
        const val MESSAGE_CONNECTED = 2
        const val MESSAGE_CONNECTION_FAILED = 3
    }

    override fun onBind(intent: Intent): IBinder = myBinder

    inner class MyBinder : Binder() {
        fun getService(): BluetoothService = this@BluetoothService
    }

    fun connectToDevice(address: String) {
        m_address = address
        ConnectToDevice().execute()
    }

    fun sendCommand(input: String) {
        m_bluetoothSocket?.outputStream?.write(input.toByteArray())
    }

    // 데이터 수신 시작
    fun startReceiving() {
        receiveThread = ReceiveThread(m_bluetoothSocket?.inputStream)
        receiveThread?.start()
    }

    private inner class ReceiveThread(private val inputStream: InputStream?) : Thread() {
        override fun run() {
            val buffer = ByteArray(1024)
            val accumulatedData = StringBuilder()

            while (m_isConnected) {
                try {
                    val bytes = inputStream?.read(buffer) ?: -1
                    if (bytes > 0) {
                        val incoming = String(buffer, 0, bytes)
                        accumulatedData.append(incoming)
                        
                        while (true) {
                            val content = accumulatedData.toString()
                            var keywordFound = false

                            if (content.contains("OCCUPIED")) {
                                handler?.obtainMessage(MESSAGE_READ, "OCCUPIED")?.sendToTarget()
                                val index = accumulatedData.indexOf("OCCUPIED")
                                accumulatedData.delete(0, index + "OCCUPIED".length)
                                keywordFound = true
                            } else if (content.contains("EMPTY")) {
                                handler?.obtainMessage(MESSAGE_READ, "EMPTY")?.sendToTarget()
                                val index = accumulatedData.indexOf("EMPTY")
                                accumulatedData.delete(0, index + "EMPTY".length)
                                keywordFound = true
                            } else if (content.contains("SUSPICIOUS")) {
                                handler?.obtainMessage(MESSAGE_READ, "SUSPICIOUS")?.sendToTarget()
                                val index = accumulatedData.indexOf("SUSPICIOUS")
                                accumulatedData.delete(0, index + "SUSPICIOUS".length)
                                keywordFound = true
                            } else if (content.contains("AWAY")) {
                                handler?.obtainMessage(MESSAGE_READ, "AWAY")?.sendToTarget()
                                val index = accumulatedData.indexOf("AWAY")
                                accumulatedData.delete(0, index + "AWAY".length)
                                keywordFound = true
                            } else if (content.contains("BACK")) {
                                handler?.obtainMessage(MESSAGE_READ, "BACK")?.sendToTarget()
                                val index = accumulatedData.indexOf("BACK")
                                accumulatedData.delete(0, index + "BACK".length)
                                keywordFound = true
                            } else if (content.contains("ON")) {
                                handler?.obtainMessage(MESSAGE_READ, "ON")?.sendToTarget()
                                val index = accumulatedData.indexOf("ON")
                                accumulatedData.delete(0, index + "ON".length)
                                keywordFound = true
                            } else if (content.contains("OFF")) {
                                handler?.obtainMessage(MESSAGE_READ, "OFF")?.sendToTarget()
                                val index = accumulatedData.indexOf("OFF")
                                accumulatedData.delete(0, index + "OFF".length)
                                keywordFound = true
                            }

                            if (!keywordFound) {
                                break
                            }
                        }

                        // Safety: Prevent buffer from growing too large if no keywords are found
                        if (accumulatedData.length > 2048) {
                            accumulatedData.delete(0, accumulatedData.length - 100)
                        }
                    } else if (bytes == -1) {
                         // End of stream
                         break
                    }
                } catch (e: IOException) {
                    m_isConnected = false
                    break
                }
            }
        }
    }

    private inner class ConnectToDevice : AsyncTask<Void, Void, Boolean>() {
        @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
        override fun doInBackground(vararg p0: Void?): Boolean {
            var connectSuccess = true
            try {
                if (m_bluetoothSocket == null || !m_isConnected) {
                    val device = BluetoothAdapter.getDefaultAdapter().getRemoteDevice(m_address)
                    
                    // 1. Try Insecure RFCOMM (Standard)
                    try {
                        m_bluetoothSocket = device.createInsecureRfcommSocketToServiceRecord(m_myUUID)
                        BluetoothAdapter.getDefaultAdapter().cancelDiscovery()
                        m_bluetoothSocket?.connect()
                    } catch (e: IOException) {
                        Log.e("BluetoothService", "Standard connection failed", e)
                        try {
                            m_bluetoothSocket?.close()
                        } catch (c: IOException) {}
                        
                        // 2. Fallback: Reflection Method (For older/specific devices)
                        try {
                            Log.i("BluetoothService", "Trying fallback reflection method...")
                            val m = device.javaClass.getMethod("createRfcommSocket", Int::class.javaPrimitiveType)
                            m_bluetoothSocket = m.invoke(device, 1) as BluetoothSocket
                            m_bluetoothSocket?.connect()
                        } catch (e2: Exception) {
                            Log.e("BluetoothService", "Fallback connection failed", e2)
                            connectSuccess = false
                        }
                    }
                }
            } catch (e: IOException) {
                connectSuccess = false
            }
            return connectSuccess
        }

        override fun onPostExecute(result: Boolean) {
            super.onPostExecute(result)
            if (!result) {
                Log.e("BluetoothService", "Could not connect to device")
                m_isConnected = false
                handler?.obtainMessage(MESSAGE_CONNECTION_FAILED)?.sendToTarget()
            } else {
                m_isConnected = true
                Log.i("BluetoothService", "Connected!")
                handler?.obtainMessage(MESSAGE_CONNECTED)?.sendToTarget()
                startReceiving()
            }
        }
    }
}