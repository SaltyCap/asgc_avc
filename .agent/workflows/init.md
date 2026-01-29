---
description: Initialize and start the ASGC Autonomous Vehicle Challenge system
---

# ASGC AVC Initialization Workflow

This workflow initializes and starts the complete autonomous vehicle control system.

## Prerequisites Check

1. Verify you're on a Raspberry Pi with I2C and PWM capabilities
2. Ensure hardware is connected:
   - 2x Brushless motors with ESCs
   - AS5600 magnetic encoder on I2C
   - PWM pins (GPIO 12, 13)

## System Initialization

// turbo
3. Set I2C speed to 400kHz
```bash
sudo dtparam i2c_arm_baudrate=400000
```

// turbo
4. Build the C motor control code
```bash
cd /home/asgc/asgc_avc/c_code && make
```

// turbo
5. Create Python virtual environment (if not exists)
```bash
cd /home/asgc/asgc_avc/web_server && python3 -m venv venv
```

// turbo
6. Install Python dependencies
```bash
cd /home/asgc/asgc_avc/web_server && source venv/bin/activate && pip install -r requirements.txt
```

// turbo
7. Generate SSL certificates for HTTPS (required for microphone access)
```bash
cd /home/asgc/asgc_avc/web_server && openssl req -x509 -newkey rsa:4096 -nodes -out cert.pem -keyout key.pem -days 365 -subj "/C=US/ST=Arkansas/L=Conway/O=ASGC/CN=localhost"
```

// turbo
8. Check for Vosk speech recognition model
```bash
cd /home/asgc/asgc_avc/web_server && ls -d model 2>/dev/null || echo "Model not found - will need to download"
```

9. If Vosk model is missing, download it (600MB, may take several minutes)
```bash
cd /home/asgc/asgc_avc/web_server && wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && unzip vosk-model-small-en-us-0.15.zip && mv vosk-model-small-en-us-0.15 model && rm vosk-model-small-en-us-0.15.zip
```

## Start the System

// turbo
10. Start the complete system using the startup script
```bash
cd /home/asgc/asgc_avc && ./start_all.sh
```

## Access the Interfaces

After startup, access these URLs (replace `your-ip` with your Raspberry Pi's IP address):

- **Voice Control + Queue**: https://your-ip:5000/
- **Joystick Control**: https://your-ip:5000/joystick
- **Course View**: https://your-ip:5000/course

**Note**: Your browser will show a security warning for the self-signed SSL certificate. This is expected - click "Advanced" and "Proceed" to continue.

## Quick Test

11. Test voice recognition:
    - Open https://your-ip:5000/
    - Click "Start Voice Recognition"
    - Say "center" to queue navigation to center
    - Watch the command appear in the queue

12. Test manual control:
    - Open https://your-ip:5000/joystick
    - Use touch/click to control motors
    - Verify both motors respond

## Troubleshooting

If the system doesn't start:
- Check I2C is enabled: `sudo raspi-config` → Interface Options → I2C
- Verify PWM pins are available: `gpio readall`
- Check motor controller compilation: `cd c_code && make clean && make`
- Review logs in the `logs/` directory

If voice recognition doesn't work:
- Ensure you're using HTTPS (required for microphone access)
- Check browser microphone permissions
- Verify Vosk model is in `web_server/model/`

## Next Steps

After initialization:
- Calibrate wheel diameter and wheelbase (see README.md)
- Test navigation to each bucket
- Practice voice command queue usage
- Review course layout and start position
