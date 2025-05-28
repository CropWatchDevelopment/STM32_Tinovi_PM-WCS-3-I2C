
// Initialize:
pmwcs3_init(&soil, &hi2c1, 0x63);
float vals[4] = {0};

// Poll new data from sensor:
pmwcs3_new_reading(&soil);
HAL_Delay(250);
pmwcs3_get_all(&soil, vals);
