# Time Synchronization System

## Overview

The Joint Controller implements an **NTP-like time synchronization protocol** between the host computer and the Pico firmware. This allows accurate timestamping of sensor readings and movement data across the two systems.

## Why Time Synchronization?

When collecting movement data, encoder readings, and motor commands, it's essential to have a common time reference between:
- **Host**: Python application (running on computer)
- **Firmware**: C++ application (running on Pico microcontroller)

Without synchronization:
- Host timestamps use `time.time()` (Unix epoch in seconds)
- Firmware timestamps use `micros()` (microseconds since boot)
- These timestamps are **incomparable** and make correlation impossible

With synchronization:
- We calculate the time offset between the two systems
- We can convert firmware timestamps to host time or vice versa
- Data from both systems can be accurately correlated and analyzed

## NTP-Like Protocol

The implementation uses a simplified **Network Time Protocol (NTP)** approach:

```
┌─────────┐                           ┌──────────┐
│  Host   │                           │ Firmware │
└─────────┘                           └──────────┘
     │                                      │
     │ T1: Send SYNC(T1)                    │
     ├─────────────────────────────────────>│
     │                                      │ T2: Receive, note time
     │                                      │
     │                   T3: Send response  │
     │<─────────────────────────────────────┤
     │                                      │
     │ T4: Receive response                 │
```

### Timing Points

1. **T1** - Host sends SYNC command (host timestamp)
2. **T2** - Firmware receives command (firmware timestamp)
3. **T3** - Firmware sends response (same as T2 for immediate response)
4. **T4** - Host receives response (host timestamp)

### Offset Calculation

The time offset is calculated using the NTP algorithm:

```
offset = ((T2 - T1) + (T3 - T4)) / 2
```

Since T3 = T2 (immediate response), this simplifies to:

```
offset = T2 - (T1 + T4) / 2
```

### Round-Trip Time (RTT)

```
RTT = T4 - T1
```

The RTT indicates the quality of the synchronization:
- **< 10ms**: Good quality
- **10-50ms**: Fair quality
- **> 50ms**: Poor quality (retry recommended)

## Implementation

### Firmware (`joint_controller/src/core0.cpp`)

```cpp
case CMD_SYNC: {
  double T1 = 0.0;
  double T2 = 0.0;
  
  // Extract T1 from command (format: SYNC(T1))
  char* paren_start = strchr(actual_command, '(');
  if (paren_start != nullptr) {
    T1 = atof(paren_start + 1);
    T2 = micros() / 1000000.0; // Firmware time in seconds
    
    // Send response with T1 and T2
    Serial.print("EVT:SYNC_RESPONSE(");
    Serial.print(T1, 6);
    Serial.print(",");
    Serial.print(T2, 6);
    Serial.println(")");
  }
  break;
}
```

### Host (`serial_handler.py`)

```python
def synchronize_time(self) -> Dict[str, float]:
    # T1: Send SYNC command
    T1 = time.time()
    command = f"CMD:SYNC({T1:.6f})\n"
    ser.write(command.encode())
    
    # Wait for response
    line = ser.readline().decode('utf-8').strip()
    
    # T4: Record receive time
    T4 = time.time()
    
    # Parse EVT:SYNC_RESPONSE(T1_echo,T2)
    T1_echo, T2 = parse_sync_response(line)
    
    # Calculate offset
    offset = T2 - (T1 + T4) / 2.0
    rtt = T4 - T1
    
    return {
        "success": True,
        "offset": offset,
        "rtt": rtt,
        "T1": T1,
        "T2": T2,
        "T3": T2,
        "T4": T4
    }
```

### UI (`scripts.js` + `index.html`)

A button in the Status Messages panel allows manual synchronization:

```javascript
function syncTime() {
    $.ajax({
        url: '/sync_time',
        type: 'POST',
        data: JSON.stringify({ joint: selectedJoint }),
        success: function(response) {
            displaySyncResults(response.sync_data);
        }
    });
}
```

Results are displayed in a dedicated panel showing:
- Time offset (milliseconds)
- Round-trip time (milliseconds)
- Quality indicator (good/fair/poor)
- Detailed timing breakdown (T1, T2, T3, T4)
- NTP formula used

## Usage

1. **Connect to a joint** via the UI
2. **Click the clock icon** (🕐) in the Status Messages header
3. **Review the results** in the Time Synchronization panel:
   - Check the offset value
   - Verify RTT is low (< 10ms for best results)
   - Confirm quality is "GOOD"
4. **Retry if needed** - Multiple syncs can improve accuracy

## Precision & Limitations

### Achievable Precision
- **USB CDC Serial**: ~1-5ms typical latency
- **Time Resolution**: Microsecond (6 decimal places)
- **Expected Offset Accuracy**: ±5-10ms

### Limitations
1. **Asymmetric Latency**: Assumes equal send/receive delays (T2-T1 ≈ T4-T3)
2. **USB Jitter**: USB CDC serial has variable latency
3. **No Clock Drift Correction**: Offset is static; clocks may drift over time
4. **Single Sample**: No statistical averaging (could be added for improvement)

### Future Improvements
- Multiple sync samples with statistical averaging
- Periodic automatic re-synchronization
- Clock drift detection and compensation
- Hardware timestamping (if available)

## Protocol Specification

### Command Format
```
CMD:SYNC(<T1>)
```
- `<T1>`: Host timestamp in seconds (double, 6 decimal places)

Example: `CMD:SYNC(1728500000.123456)`

### Response Format
```
EVT:SYNC_RESPONSE(<T1_echo>,<T2>)
```
- `<T1_echo>`: Echoed T1 from command
- `<T2>`: Firmware timestamp in seconds (double, 6 decimal places)

Example: `EVT:SYNC_RESPONSE(1728500000.123456,3.456789)`

## References

- [Network Time Protocol (NTP) - RFC 5905](https://datatracker.ietf.org/doc/html/rfc5905)
- [PROTOCOL.md](../firmware/joint_controller/PROTOCOL.md) - Serial protocol specification

