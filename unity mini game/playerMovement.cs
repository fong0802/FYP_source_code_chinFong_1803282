using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class playerMovement : MonoBehaviour
{
    private bleRead bleSensor;
    private BoxCollider2D coll;
    private Rigidbody2D body;

    [SerializeField] private float jumpHeight = 4f;
    [SerializeField] private LayerMask jumpableGround;
    [SerializeField] private float thres = 2000f;
    [SerializeField] private Text capacitanceChangeText;

    public ushort sensorData;
    private ushort sensorDataCurrent;

    private void Start()
    {
        bleSensor = GetComponent<bleRead>();
        coll = GetComponent<BoxCollider2D>();
        body = GetComponent<Rigidbody2D>();
    }

    // Update is called once per frame
    private void Update()
    {
        sensorData = bleSensor.sensor_data;

        if(body.velocity.y == 0)
        {
            sensorDataCurrent = sensorData;
            capacitanceChangeText.text = "Capacitance Change: " + sensorDataCurrent;
        }

        if (sensorDataCurrent > thres && IsGrounded() && body.velocity.y == 0)
        {
            body.velocity = new Vector3(0, (sensorDataCurrent / 1000) * jumpHeight, 0);
        }
    }

    private bool IsGrounded()
    {
        return Physics2D.BoxCast(coll.bounds.center, coll.bounds.size, 0f, Vector2.down, 0.1f, jumpableGround);
    } 
}
