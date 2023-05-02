using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;

using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class bleRead : MonoBehaviour
{
    private string device_id = "BluetoothLE#BluetoothLEf8:59:71:70:ef:9e-e0:5a:1b:75:9f:ee";
    private string service_uuid = "00001810-0000-1000-8000-00805f9b34fb";
    private string characteristic_uuid = "00002a35-0000-1000-8000-00805f9b34fb";
    
    private string device_name;
    private bool sensor_connected = false;

    public ushort sensor_data;

    // Update is called once per frame
    void Update()
    {
        if (!sensor_connected)
        {
            BleApi.StartDeviceScan();
            BleApi.DeviceUpdate device = new BleApi.DeviceUpdate();

            BleApi.PollDevice(ref device, false);

            if (device.id == device_id)
            {
                device_name = device.name;
                BleApi.StopDeviceScan();
                BleApi.SubscribeCharacteristic(device_id, service_uuid, characteristic_uuid, false);
                sensor_connected = true;
                Debug.Log("Connected device -> " + device.name);
            }
        }

        if (sensor_connected)
        {
            BleApi.BLEData data = new BleApi.BLEData();
             
            if(BleApi.PollData(out data, false))
            {
                byte[] temp = {data.buf[1], data.buf[0]};
                sensor_data = BitConverter.ToUInt16(temp, 0);
            }
        }
    }

    private void OnApplicationQuit()
    {
        BleApi.Quit();
    }
}
